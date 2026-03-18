"""
AFCS-105 LINKER — FCS 연결 단말기 (PC ↔ STM32 블루투스 브릿지)

표적 좌표를 암호화해서 FCS로 전송하고, 사격제원 응답을 수신/표시.
스트레스 테스트(N회 랜덤 전송 + 통계)와 오류 주입(CRC/ETX/CMD 변조) 지원.

프로토콜: [STX][CMD][SALT][LEN][PAYLOAD...][CRC32(4B)][ETX]
암호화 — Rolling XOR, session_key = MASTER_KEY ^ SALT
CRC-32 — MPEG-2 (poly 0x04C11DB7, 빅엔디안 워드 패킹)
"""

import customtkinter as ctk
import serial
import serial.tools.list_ports
import threading
import time
import random
import re

# 프로토콜 상수 (STM32 fcs_core.h와 일치해야 함)
STX = 0x02
ETX = 0x03
MASTER_KEY = 0xA5

# 송신 명령 (Python -> STM32)
CMD_TARGET = 0xA1
CMD_STATUS = 0xC1

# 수신 명령 (STM32 -> Python)
CMD_ACK = 0xA2
CMD_ERR = 0xE1
CMD_STATUS_ACK = 0xC2

# 요청 CMD -> 허용 응답 CMD 매핑
VALID_RESPONSES = {
    CMD_TARGET: (CMD_ACK, CMD_ERR),
    CMD_STATUS: (CMD_STATUS_ACK, CMD_ERR),
}

# 타이밍
RESPONSE_TIMEOUT_S = 5.0
MAX_RETRIES = 2
SERIAL_POLL_INTERVAL_S = 0.05

# 입력 유효성 범위
VALID_ZONES = tuple(range(1, 61))       # UTM 1~60
VALID_BANDS = tuple("CDEFGHJKLMNPQRSTUVWX")  # UTM 밴드 (I, O 제외)
EASTING_RANGE = (100000, 999999)
NORTHING_RANGE = (0, 9999999)           # 전 지구 UTM northing
ALTITUDE_RANGE = (0, 9999)

# 스트레스 테스트 기본값 (STATUS_REQ 조회 실패 시 사용)
STRESS_DEFAULT_E = 333712
STRESS_DEFAULT_N = 4132894
STRESS_RADIUS_M = 7000

# 에러 코드 설명
ERROR_DESCRIPTIONS = {
    "CRC":       "CRC 체크섬 불일치",
    "PARSE":     "STM32 페이로드 파싱 실패",
    "RANGE":     "사거리 범위 초과 (1.1~11.3km)",
    "CHARGE":    "현재 장약으로 도달 불가",
    "CALC":      "사표 조회 오류",
    "TRAVERSE":  "전이한계 초과 (재방열 필요)",
    "ZONE":      "UTM 존 불일치 (재투영 필요)",
    "NOT READY": "STM32가 사격 가능한 상태가 아님",
    "TIMEOUT":   "타임아웃 (응답 없음)",
}

# 응답 필드 정규식: AZ:3200.5 같은 KEY:VALUE 쌍 매칭
RESPONSE_PATTERN = re.compile(r'([A-Z]+):([\d.]+)')



# CRC-32/MPEG-2 (다항식 0x04C11DB7, STM32 HW CRC 유닛과 동일)
def calc_crc32(data: bytes) -> int:
    """빅엔디안 워드 패킹 CRC-32 계산 (STM32F401 HW CRC와 일치)."""
    crc = 0xFFFFFFFF
    length = len(data)
    words = length // 4
    tail = length % 4

    # 32비트 워드 단위 처리 (빅엔디안 패킹)
    for i in range(words):
        idx = i * 4
        word = (data[idx] << 24) | (data[idx+1] << 16) | (data[idx+2] << 8) | data[idx+3]
        crc ^= word
        for _ in range(32):
            if crc & 0x80000000:
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc <<= 1
            crc &= 0xFFFFFFFF

    # 잔여 바이트 처리 (0 패딩, MSB 정렬)
    if tail > 0:
        word = 0
        base = words * 4
        for j in range(tail):
            word |= data[base + j] << (24 - 8 * j)
        crc ^= word
        for _ in range(32):
            if crc & 0x80000000:
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc <<= 1
            crc &= 0xFFFFFFFF

    return crc


# 메인 애플리케이션
class FCSClientApp(ctk.CTk):
    """AFCS-105 LINKER 메인 윈도우."""

    def __init__(self):
        super().__init__()
        self.title("AFCS-105 LINKER")
        self.geometry("620x720")
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        # 시리얼 상태
        self.ser = None
        self.connected = False
        self.serial_lock = threading.Lock()

        # 수신 리스너 스레드
        self.running = True
        self.rx_buffer = bytearray()

        # 하트비트 (BT 연결 유지)
        self.heartbeat_stop = threading.Event()

        # 응답 동기화 (사격 임무 및 스트레스 테스트)
        self.response_event = threading.Event()
        self.last_response = None  # (is_ack: bool, text: str)
        self.expected_salt = None  # 요청 salt -> 응답 매칭용
        self.expected_cmd = None   # 요청 CMD -> 응답 CMD 검증용
        self.in_flight = threading.Lock()  # 단일 요청 직렬화

        self._build_ui()
        self._start_listener()

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # UI 구성
    def _build_ui(self):
        """전체 UI 레이아웃 구성."""
        self._build_connection_bar()
        self._build_tabs()
        self._build_log_panel()

    def _build_connection_bar(self):
        """상단 연결 바: 포트 선택, 스캔, 연결, 상태."""
        bar = ctk.CTkFrame(self)
        bar.pack(fill="x", padx=10, pady=(10, 5))

        self.port_var = ctk.StringVar()
        self.port_combo = ctk.CTkComboBox(bar, variable=self.port_var, width=110)
        self.port_combo.pack(side="left", padx=(10, 3))
        self._refresh_ports()

        ctk.CTkButton(bar, text="Scan", width=50,
                       command=self._refresh_ports).pack(side="left", padx=3)

        self.btn_conn = ctk.CTkButton(bar, text="CONNECT", width=90,
                                       command=self._toggle_connection)
        self.btn_conn.pack(side="left", padx=3)

        self.status_label = ctk.CTkLabel(bar, text="OFFLINE",
                                          text_color="#ff4444")
        self.status_label.pack(side="right", padx=10)

    def _build_tabs(self):
        """탭 뷰: FIRE MISSION / STRESS TEST."""
        self.tabview = ctk.CTkTabview(self, height=210)
        self.tabview.pack(fill="x", padx=10, pady=5)

        self.tab_fire = self.tabview.add("FIRE MISSION")
        self.tab_stress = self.tabview.add("STRESS TEST")

        self._build_fire_tab()
        self._build_stress_tab()

    def _build_fire_tab(self):
        """FIRE MISSION 탭: 좌표 입력 + 결과 표시."""
        mono = ctk.CTkFont(family="Consolas", size=12)
        mono_bold = ctk.CTkFont(family="Consolas", size=12, weight="bold")

        # 세로 가운데 정렬용 wrapper
        center = ctk.CTkFrame(self.tab_fire, fg_color="transparent")
        center.place(relx=0.5, rely=0.5, anchor="center")

        # --- 입력: 한 줄 ---
        row0 = ctk.CTkFrame(center)
        row0.pack(pady=(0, 5))

        self.zone_var = ctk.StringVar(value="52")
        self.band_var = ctk.StringVar(value="S")
        self.east_var = ctk.StringVar(value="340000")
        self.north_var = ctk.StringVar(value="4139000")
        self.alt_var = ctk.StringVar(value="100")

        for label, var, w in [("ZONE:", self.zone_var, 32), ("BAND:", self.band_var, 28),
                               ("EAST:", self.east_var, 68), ("NORTH:", self.north_var, 78),
                               ("ALT:", self.alt_var, 45)]:
            ctk.CTkLabel(row0, text=label, font=mono,
                          text_color="#888888").pack(side="left", padx=(5, 1))
            ctk.CTkEntry(row0, textvariable=var, width=w, font=mono,
                          height=28, justify="center").pack(side="left")

        self.btn_transmit = ctk.CTkButton(
            row0, text="SEND", command=self._on_transmit,
            width=65, height=28, font=mono_bold)
        self.btn_transmit.pack(side="left", padx=(8, 3))

        # --- 응답: 상태 + 제원 (1줄) / 에러 설명 (2줄째) ---
        res = ctk.CTkFrame(center)
        res.pack(pady=(5, 0))

        self.result_status_label = ctk.CTkLabel(
            res, text="STANDBY", text_color="gray",
            font=ctk.CTkFont(family="Consolas", size=12, weight="bold"))
        self.result_status_label.pack(side="left", padx=(5, 5))

        self.result_error_desc = ctk.CTkLabel(
            center, text="", font=mono, text_color="#aaaaaa")

        self.result_vars = {}
        for key in ["AZ", "EL", "CHG", "TOF", "DIST"]:
            ctk.CTkLabel(res, text=f"{key}:", font=mono,
                          text_color="#888888").pack(side="left", padx=(5, 0))
            var = ctk.StringVar(value="---")
            ctk.CTkLabel(res, textvariable=var, font=mono_bold).pack(side="left")
            self.result_vars[key] = var

    def _build_stress_tab(self):
        """STRESS TEST 탭: 설정 + 진행률 + 통계."""
        mono = ctk.CTkFont(family="Consolas", size=12)
        mono_bold = ctk.CTkFont(family="Consolas", size=12, weight="bold")

        # --- Row 1: 반복횟수, 간격, START, STOP (한 줄) ---
        row1 = ctk.CTkFrame(self.tab_stress)
        row1.pack(pady=(3, 3))

        self.iter_var = ctk.StringVar(value="100")
        self.interval_var = ctk.StringVar(value="200")

        ctk.CTkLabel(row1, text="N:", font=mono,
                      text_color="#888888").pack(side="left", padx=(5, 1))
        ctk.CTkEntry(row1, textvariable=self.iter_var, width=50, font=mono,
                      height=28, justify="center").pack(side="left")

        ctk.CTkLabel(row1, text="INT:", font=mono,
                      text_color="#888888").pack(side="left", padx=(8, 1))
        ctk.CTkEntry(row1, textvariable=self.interval_var, width=45, font=mono,
                      height=28, justify="center").pack(side="left")
        ctk.CTkLabel(row1, text="ms", font=mono,
                      text_color="#888888").pack(side="left", padx=(1, 0))

        self.btn_start_test = ctk.CTkButton(
            row1, text="START", command=self._on_start_stress,
            width=65, height=28, font=mono_bold, fg_color="#2d8f2d")
        self.btn_start_test.pack(side="left", padx=(8, 3))

        self.btn_stop_test = ctk.CTkButton(
            row1, text="STOP", command=self._on_stop_stress,
            width=55, height=28, font=mono_bold, fg_color="#aa3333",
            state="disabled")
        self.btn_stop_test.pack(side="left", padx=3)

        # --- Row 2: 오류 주입 체크박스 ---
        inj_row = ctk.CTkFrame(self.tab_stress)
        inj_row.pack(pady=(2, 2))

        inj_font = ctk.CTkFont(family="Consolas", size=11)
        ctk.CTkLabel(inj_row, text="INJECT:", font=inj_font,
                      text_color="#888888").pack(side="left", padx=(5, 3))

        self.inj_crc_var = ctk.BooleanVar(value=False)
        self.inj_etx_var = ctk.BooleanVar(value=False)
        self.inj_cmd_var = ctk.BooleanVar(value=False)

        ctk.CTkCheckBox(inj_row, text="CRC ERR", variable=self.inj_crc_var,
                          font=inj_font, width=30, height=20,
                          checkbox_width=16, checkbox_height=16).pack(side="left", padx=4)
        ctk.CTkCheckBox(inj_row, text="NO ETX", variable=self.inj_etx_var,
                          font=inj_font, width=30, height=20,
                          checkbox_width=16, checkbox_height=16).pack(side="left", padx=4)
        ctk.CTkCheckBox(inj_row, text="BAD CMD", variable=self.inj_cmd_var,
                          font=inj_font, width=30, height=20,
                          checkbox_width=16, checkbox_height=16).pack(side="left", padx=4)

        # --- Row 3: 진행률 바 ---
        row2 = ctk.CTkFrame(self.tab_stress)
        row2.pack(fill="x", padx=15, pady=2)

        ctk.CTkLabel(row2, text="PROGRESS:", font=mono,
                      text_color="#888888").pack(side="left", padx=(5, 3))
        self.progress_bar = ctk.CTkProgressBar(row2, height=14)
        self.progress_bar.pack(side="left", expand=True, fill="x", padx=(0, 5))
        self.progress_bar.set(0)

        self.progress_label = ctk.CTkLabel(row2, text="0/0", font=mono)
        self.progress_label.pack(side="left")

        # --- Row 3~4: 통계 2줄 ---
        stats = ctk.CTkFrame(self.tab_stress)
        stats.pack(pady=(2, 3))

        self.stats_comm_var = ctk.StringVar(
            value="ACK: 0   TIMEOUT: 0 (AVG:--- MIN:--- MAX:---)")
        ctk.CTkLabel(stats, textvariable=self.stats_comm_var, font=mono,
                      text_color="#cccccc").pack()

        self.stats_fire_var = ctk.StringVar(
            value="ABLE: 0   UNABLE: 0 (RANGE:0 CHARGE:0 TRAVERSE:0)")
        ctk.CTkLabel(stats, textvariable=self.stats_fire_var, font=mono,
                      text_color="#cccccc").pack()

        # 스트레스 테스트 상태
        self.stress_running = False
        self.stress_stop_event = threading.Event()

    def _build_log_panel(self):
        """하단 통신 로그 패널."""
        log_frame = ctk.CTkFrame(self)
        log_frame.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        header = ctk.CTkFrame(log_frame)
        header.pack(fill="x", padx=10, pady=(5, 0))
        ctk.CTkButton(header, text="CLR", width=40,
                       command=self._clear_log).pack(side="right")

        self.log_text = ctk.CTkTextbox(log_frame, wrap="none",
                                        font=ctk.CTkFont(family="Consolas", size=12))
        self.log_text.pack(fill="both", expand=True, padx=10, pady=5)

    # 연결 관리
    def _refresh_ports(self):
        """사용 가능한 COM 포트를 스캔하여 콤보박스에 표시."""
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.configure(values=ports)
        if ports:
            self.port_var.set(ports[0])

    def _toggle_connection(self):
        """시리얼 포트 연결/해제 토글."""
        if not self.connected:
            port = self.port_var.get()
            if not port:
                self._log("포트를 선택하세요", "SYS")
                return
            self.btn_conn.configure(state="disabled")
            threading.Thread(target=self._connect_worker, args=(port,),
                             daemon=True).start()
        else:
            self._disconnect()

    def _connect_worker(self, port: str):
        """백그라운드: BT SPP 연결 재시도 (Windows 세마포 타임아웃 대응)."""
        max_retries = 3
        for attempt in range(1, max_retries + 1):
            try:
                ser = serial.Serial(port, 9600, timeout=0.1)
                self.ser = ser
                self.connected = True
                self.after(0, lambda: self.btn_conn.configure(text="DISCONNECT", state="normal"))
                self.after(0, lambda: self.status_label.configure(text="ONLINE", text_color="#44ff44"))
                self.after(0, self._log, f"{port} 연결 완료", "SYS")
                self.after(0, self._start_heartbeat)
                return
            except serial.SerialException as e:
                self.after(0, self._log,
                           f"연결 재시도 {attempt}/{max_retries}: {e}", "SYS")
                if attempt < max_retries:
                    time.sleep(1.0)
        self.after(0, self._log, f"{max_retries}회 시도 후 연결 실패", "ERR")
        self.after(0, lambda: self.btn_conn.configure(state="normal"))

    def _disconnect(self):
        """시리얼 포트 안전 해제."""
        self._stop_heartbeat()
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
        self.connected = False
        self.btn_conn.configure(text="CONNECT")
        self.status_label.configure(text="OFFLINE", text_color="#ff4444")
        self._log("연결 해제", "SYS")

    def _start_heartbeat(self):
        """BT 하트비트 스레드 시작 (3초마다 null 바이트)."""
        self.heartbeat_stop.clear()
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()

    def _stop_heartbeat(self):
        """BT 하트비트 스레드 중지."""
        self.heartbeat_stop.set()

    def _heartbeat_loop(self):
        """백그라운드: FCS 연결 감지용 0x00을 3초마다 전송."""
        while not self.heartbeat_stop.wait(timeout=3.0):
            with self.serial_lock:
                if self.ser and self.ser.is_open:
                    try:
                        self.ser.write(b'\x00')
                    except serial.SerialException:
                        break

    # 로깅
    def _log(self, msg: str, tag: str = "SYS"):
        """통신 로그에 메시지 추가."""
        line = f"[{tag}] {msg}\n"
        self.log_text.insert("end", line)
        self.log_text.see("end")

    def _clear_log(self):
        """통신 로그 전체 삭제."""
        self.log_text.delete("1.0", "end")

    # 패킷 구성 및 전송
    def _build_packet(self, cmd_id: int, payload_str: str) -> tuple[bytes, int]:
        """암호화된 프로토콜 패킷 생성. (패킷 bytes, salt) 반환."""
        raw = payload_str.encode("utf-8")
        salt = random.randint(0, 255)
        session_key = MASTER_KEY ^ salt

        encrypted = bytearray()
        for i, byte in enumerate(raw):
            encrypted.append(byte ^ ((session_key + i) & 0xFF))

        length = len(encrypted)
        crc_data = bytes([cmd_id, salt, length]) + bytes(encrypted)
        crc = calc_crc32(crc_data)

        packet = bytearray([STX, cmd_id, salt, length])
        packet.extend(encrypted)
        # CRC-32 빅엔디안 (4바이트)
        packet.append((crc >> 24) & 0xFF)
        packet.append((crc >> 16) & 0xFF)
        packet.append((crc >> 8) & 0xFF)
        packet.append(crc & 0xFF)
        packet.append(ETX)
        return bytes(packet), salt

    def _send_packet(self, packet: bytes) -> bool:
        """패킷 전송 (serial_lock 보호). 실패 시 False."""
        with self.serial_lock:
            if not self.ser or not self.ser.is_open:
                return False
            try:
                self.ser.write(packet)
                return True
            except serial.SerialException:
                return False

    def _query_battery_position(self) -> tuple[int, str, int, int] | None:
        """STATUS_REQ로 STM32에 포대 좌표 조회. 성공 시 (zone, band, e, n), 실패 시 None."""
        packet, salt = self._build_packet(CMD_STATUS, "")
        self.response_event.clear()
        self.last_response = None
        self.expected_salt = salt
        self.expected_cmd = CMD_STATUS

        if not self._send_packet(packet):
            return None

        if not self.response_event.wait(timeout=RESPONSE_TIMEOUT_S):
            return None

        is_ack, text = self.last_response
        if not is_ack:
            return None

        # "STATUS:READY,52,S,333712,4132894,100" 파싱
        try:
            parts = text.split(",")
            zone = int(parts[1])
            band = parts[2]
            easting = int(parts[3])
            northing = int(parts[4])
            return (zone, band, easting, northing)
        except (IndexError, ValueError):
            return None

    # 시리얼 수신 리스너
    def _start_listener(self):
        """백그라운드 시리얼 수신 리스너 스레드 시작."""
        t = threading.Thread(target=self._listener_loop, daemon=True)
        t.start()

    def _listener_loop(self):
        """백그라운드: 시리얼 바이트를 읽어 바이너리 응답 패킷을 디코딩.

        패킷 포맷: [STX][CMD][SALT][LEN][암호화 페이로드...][CRC32(4B)][ETX]
        패킷 외 바이트(디버그 출력 등)는 무시.
        """
        while self.running:
            if self.connected and self.ser:
                try:
                    with self.serial_lock:
                        if self.ser.in_waiting:
                            data = self.ser.read(self.ser.in_waiting)
                        else:
                            data = None
                    if data:
                        self.rx_buffer.extend(data)
                        self._process_rx_buffer()
                except Exception as e:
                    self.after(0, self._log, f"리스너 오류: {e}", "ERR")
            time.sleep(SERIAL_POLL_INTERVAL_S)

    def _process_rx_buffer(self):
        """수신 버퍼에서 완전한 바이너리 패킷을 추출하여 처리."""
        while True:
            # STX 탐색
            stx_pos = self.rx_buffer.find(STX)
            if stx_pos < 0:
                self.rx_buffer.clear()
                return
            if stx_pos > 0:
                self.rx_buffer = self.rx_buffer[stx_pos:]

            # 최소 프레임: STX(1) + CMD(1) + SALT(1) + LEN(1) + CRC32(4) + ETX(1) = 9
            if len(self.rx_buffer) < 9:
                return

            cmd  = self.rx_buffer[1]
            salt = self.rx_buffer[2]
            plen = self.rx_buffer[3]

            # 전체 프레임 길이: STX(1)+CMD(1)+SALT(1)+LEN(1)+plen+CRC(4)+ETX(1) = plen + 9
            frame_len = plen + 9
            if len(self.rx_buffer) < frame_len:
                return  # 아직 불완전

            # ETX 확인
            if self.rx_buffer[frame_len - 1] != ETX:
                # 잘못된 프레임 — STX 건너뛰고 재탐색
                self.rx_buffer = self.rx_buffer[1:]
                continue

            # 프레임 추출
            frame = bytes(self.rx_buffer[:frame_len])
            self.rx_buffer = self.rx_buffer[frame_len:]

            # CRC-32 검증
            enc_payload = frame[4:4 + plen]
            crc_data = frame[1:4 + plen]  # CMD + SALT + LEN + payload
            crc_calc = calc_crc32(crc_data)
            crc_recv = (frame[4 + plen] << 24) | (frame[5 + plen] << 16) | \
                       (frame[6 + plen] << 8)  | frame[7 + plen]

            if crc_calc != crc_recv:
                self.after(0, self._log,
                           f"CRC 불일치 (calc=0x{crc_calc:08X} recv=0x{crc_recv:08X})", "ERR")
                continue

            # 복호화
            session_key = MASTER_KEY ^ salt
            decrypted = bytearray()
            for i, b in enumerate(enc_payload):
                decrypted.append(b ^ ((session_key + i) & 0xFF))
            payload_text = decrypted.decode(errors="replace")

            # 로그 (hex 축약: 앞4+뒤4 바이트만)
            self.after(0, self._log,
                       f"[{self._hex_brief(frame)}] {payload_text}", "RX")

            # 요청-응답 상관관계 검증 (salt + cmd)
            if self.expected_salt is not None and salt != self.expected_salt:
                self.after(0, self._log,
                           f"Salt 불일치 (expected=0x{self.expected_salt:02X} recv=0x{salt:02X})", "ERR")
                continue
            allowed = VALID_RESPONSES.get(self.expected_cmd, ())
            if allowed and cmd not in allowed:
                self.after(0, self._log,
                           f"예상 외 CMD 0x{cmd:02X} (요청 0x{self.expected_cmd:02X})", "ERR")
                continue

            # 응답 분류
            is_ack = (cmd in (CMD_ACK, CMD_STATUS_ACK))
            self.last_response = (is_ack, payload_text)
            self.response_event.set()

    # 응답 파싱
    @staticmethod
    def _hex_brief(data: bytes) -> str:
        """패킷 바이트를 hex 문자열로 축약 (앞4..뒤4)."""
        h = data.hex().upper()
        if len(h) > 16:
            return h[:8] + ".." + h[-8:]
        return h

    @staticmethod
    def _parse_response(text: str) -> dict:
        """응답 페이로드에서 KEY:VALUE 쌍을 dict로 파싱."""
        return {m.group(1): m.group(2) for m in RESPONSE_PATTERN.finditer(text)}

    @staticmethod
    def _classify_error(text: str) -> str:
        """ERR 페이로드에서 에러 유형 키워드 추출."""
        upper = text.upper()
        if "CRC" in upper:
            return "CRC"
        if "RANGE" in upper:
            return "RANGE"
        if "CHARGE" in upper:
            return "CHARGE"
        if "CALC" in upper:
            return "CALC"
        if "TRAVERSE" in upper:
            return "TRAVERSE"
        if "ZONE" in upper:
            return "ZONE"
        if "PARSE" in upper:
            return "PARSE"
        if "NOT READY" in upper:
            return "NOT READY"
        return "UNKNOWN"

    # 입력 유효성 검증
    def _validate_inputs(self) -> str | None:
        """좌표 입력 유효성 검증. 오류 시 메시지 반환, 정상이면 None."""
        try:
            zone = int(self.zone_var.get())
        except ValueError:
            return "Zone은 정수여야 합니다"
        if zone not in VALID_ZONES:
            return "Zone 범위: 1~60"

        band = self.band_var.get().upper()
        if band not in VALID_BANDS:
            return "Band 범위: C~X (I, O 제외)"

        try:
            easting = int(self.east_var.get())
        except ValueError:
            return "Easting은 정수여야 합니다"
        if not (EASTING_RANGE[0] <= easting <= EASTING_RANGE[1]):
            return f"Easting 범위: {EASTING_RANGE[0]}~{EASTING_RANGE[1]}"

        try:
            northing = int(self.north_var.get())
        except ValueError:
            return "Northing은 정수여야 합니다"
        if not (NORTHING_RANGE[0] <= northing <= NORTHING_RANGE[1]):
            return f"Northing 범위: {NORTHING_RANGE[0]}~{NORTHING_RANGE[1]}"

        try:
            alt = int(self.alt_var.get())
        except ValueError:
            return "Altitude는 정수여야 합니다"
        if not (ALTITUDE_RANGE[0] <= alt <= ALTITUDE_RANGE[1]):
            return f"Altitude 범위: {ALTITUDE_RANGE[0]}~{ALTITUDE_RANGE[1]}"

        return None

    # FIRE MISSION 탭 로직
    def _on_transmit(self):
        """SEND 버튼 클릭 처리 — 유효성 검증, 전송, 응답 대기."""
        if not self.connected:
            self._log("연결되지 않음", "SYS")
            return
        if not self.in_flight.acquire(blocking=False):
            self._log("요청 처리 중", "SYS")
            return

        err = self._validate_inputs()
        if err:
            self._log(f"입력 오류: {err}", "SYS")
            self._display_fire_status("INVALID", "#ff4444", err)
            self.in_flight.release()
            return

        # 이전 결과 초기화
        self._display_fire_status("TRANSMITTING...", "#ffaa00")
        self.btn_transmit.configure(state="disabled")

        # 백그라운드 스레드에서 실행
        threading.Thread(target=self._transmit_worker, daemon=True).start()

    def _transmit_worker(self):
        """백그라운드: 패킷 전송 + 재시도 로직."""
        try:
            tgt_zone = int(self.zone_var.get())
            band = self.band_var.get()
            easting = int(self.east_var.get())
            northing = int(self.north_var.get())
            alt = int(self.alt_var.get())

            # 존 재투영은 MCU가 처리 — 원본 좌표 그대로 전송
            payload = f"{tgt_zone},{band},{easting},{northing},{alt}"
            packet, salt = self._build_packet(CMD_TARGET, payload)

            for attempt in range(1, MAX_RETRIES + 1):
                self.response_event.clear()
                self.last_response = None
                self.expected_salt = salt
                self.expected_cmd = CMD_TARGET

                if not self._send_packet(packet):
                    self.after(0, self._log, "전송 실패", "ERR")
                    break

                self.after(0, self._log,
                           f"[{self._hex_brief(packet)}] {payload}", "TX")

                # 응답 대기
                got_response = self.response_event.wait(timeout=RESPONSE_TIMEOUT_S)

                if got_response and self.last_response:
                    is_ack, text = self.last_response
                    if is_ack:
                        self.after(0, self._display_fire_result, text)
                        return
                    else:
                        err_type = self._classify_error(text)
                        desc = ERROR_DESCRIPTIONS.get(err_type, text)
                        self.after(0, self._display_fire_error, err_type, desc)
                        return

                # 타임아웃 — 재시도
                self.after(0, self._log,
                           f"타임아웃 (시도 {attempt}/{MAX_RETRIES})", "SYS")

            # 재시도 소진
            self.after(0, self._display_fire_error, "TIMEOUT",
                       ERROR_DESCRIPTIONS["TIMEOUT"])
        finally:
            self.in_flight.release()
            self.after(0, lambda: self.btn_transmit.configure(state="normal"))

    def _display_fire_status(self, status: str, color: str, desc: str = ""):
        """결과 상태 라벨 업데이트."""
        self.result_status_label.configure(text=status, text_color=color)
        self.result_error_desc.pack_forget()
        if desc:
            self.result_error_desc.configure(text=desc)
            self.result_error_desc.pack(pady=(0, 0))
        # ACK가 아니면 제원 클리어
        if color != "#44ff44":
            for var in self.result_vars.values():
                var.set("---")

    def _display_fire_result(self, text: str):
        """ACK 페이로드를 파싱하여 제원 필드 업데이트."""
        fields = self._parse_response(text)
        for key, var in self.result_vars.items():
            var.set(fields.get(key, "---"))
        self._display_fire_status("ACK", "#44ff44")

    def _display_fire_error(self, err_type: str, description: str):
        """에러 유형 + 설명을 결과 패널에 표시."""
        self._display_fire_status(err_type, "#ff4444", description)

    # STRESS TEST 탭 로직
    def _on_start_stress(self):
        """스트레스 테스트를 백그라운드 스레드로 시작."""
        if not self.connected:
            self._log("연결되지 않음", "SYS")
            return
        if self.stress_running:
            return
        if not self.in_flight.acquire(blocking=False):
            self._log("요청 처리 중", "SYS")
            return

        try:
            iterations = int(self.iter_var.get())
            interval_ms = int(self.interval_var.get())
        except ValueError:
            self._log("테스트 설정 오류", "SYS")
            self.in_flight.release()
            return

        if iterations < 1 or interval_ms < 50:
            self._log("반복횟수 >= 1, 간격 >= 50ms", "SYS")
            self.in_flight.release()
            return

        self.stress_running = True
        self.stress_stop_event.clear()
        self.btn_start_test.configure(state="disabled")
        self.btn_stop_test.configure(state="normal")

        self.progress_bar.set(0)
        self.progress_label.configure(text=f"0/{iterations}")
        self.stats_comm_var.set(
            "ACK: 0   TIMEOUT: 0 (AVG:--- MIN:--- MAX:---)")
        self.stats_fire_var.set(
            "ABLE: 0   UNABLE: 0 (RANGE:0 CHARGE:0 TRAVERSE:0)")

        threading.Thread(target=self._stress_worker,
                         args=(iterations, interval_ms / 1000.0),
                         daemon=True).start()

    def _on_stop_stress(self):
        """스트레스 테스트 중지 시그널."""
        self.stress_stop_event.set()

    def _stress_worker(self, iterations: int, interval_s: float):
        """백그라운드: 스트레스 테스트 실행."""
        # 포대 좌표 조회 (실패 시 기본값 사용)
        result = self._query_battery_position()
        if result:
            batt_zone, batt_band, center_e, center_n = result
        else:
            batt_zone, batt_band = 52, "S"
            center_e, center_n = STRESS_DEFAULT_E, STRESS_DEFAULT_N
            self.after(0, self._log, "포대 좌표 조회 실패, 기본값 사용", "SYS")

        self.after(0, self._log, "스트레스 테스트 시작", "SYS")

        ack_count = 0
        err_count = 0
        timeout_count = 0
        inject_count = 0
        response_times = []
        error_types = {}

        # 오류 주입 옵션 (메인 스레드 변수를 시작 시점에 스냅샷)
        inj_crc = self.inj_crc_var.get()
        inj_etx = self.inj_etx_var.get()
        inj_cmd = self.inj_cmd_var.get()
        active_injections = []
        if inj_crc:
            active_injections.append("CRC")
        if inj_etx:
            active_injections.append("ETX")
        if inj_cmd:
            active_injections.append("CMD")
        if active_injections:
            self.after(0, self._log,
                       f"오류 주입 활성: {', '.join(active_injections)} (~20%)", "SYS")

        for i in range(iterations):
            if self.stress_stop_event.is_set():
                break

            # 포진지 zone/band 고정, 좌표만 랜덤 (±반경)
            easting = center_e + random.randint(-STRESS_RADIUS_M, STRESS_RADIUS_M)
            northing = center_n + random.randint(-STRESS_RADIUS_M, STRESS_RADIUS_M)
            alt = random.randint(0, 500)

            payload = f"{batt_zone},{batt_band},{easting},{northing},{alt}"

            # 오류 주입 판정 (~20% 확률)
            inject_type = None
            if active_injections and random.random() < 0.2:
                inject_type = random.choice(active_injections)

            # 패킷 생성 (BAD CMD는 빌드 시점에 적용)
            if inject_type == "CMD":
                packet, salt = self._build_packet(0xFF, payload)
            else:
                packet, salt = self._build_packet(CMD_TARGET, payload)

            # CRC/ETX 주입 (빌드 후 변조)
            if inject_type == "CRC":
                packet = bytearray(packet)
                packet[-2] ^= 0xFF  # CRC 바이트 1개 변조
                packet = bytes(packet)
            elif inject_type == "ETX":
                packet = packet[:-1]  # ETX 제거

            if inject_type:
                inject_count += 1
                self.after(0, self._log,
                           f"[INJ] {inject_type} 주입 (#{inject_count})", "SYS")

            self.response_event.clear()
            self.last_response = None
            self.expected_salt = salt
            self.expected_cmd = CMD_TARGET

            t_start = time.perf_counter()

            if not self._send_packet(packet):
                timeout_count += 1
                continue

            self.after(0, self._log,
                       f"[{self._hex_brief(packet)}] {payload}", "TX")

            got_response = self.response_event.wait(timeout=RESPONSE_TIMEOUT_S)
            t_elapsed = (time.perf_counter() - t_start) * 1000.0  # ms

            if got_response and self.last_response:
                is_ack, text = self.last_response
                response_times.append(t_elapsed)  # 통신 성공 -> 응답시간 기록 (ACK/ERR 무관)
                if is_ack:
                    ack_count += 1
                else:
                    err_count += 1
                    et = self._classify_error(text)
                    error_types[et] = error_types.get(et, 0) + 1
            else:
                timeout_count += 1

            # UI 업데이트
            done = i + 1
            progress = done / iterations
            self.after(0, self._update_stress_progress,
                       done, iterations, progress)
            self.after(0, self._update_stress_stats,
                       ack_count, err_count, timeout_count,
                       response_times.copy(), error_types.copy())

            # 간격 대기 (중단 가능)
            self.stress_stop_event.wait(timeout=interval_s)

        # 최종 업데이트
        self.after(0, self._update_stress_stats,
                   ack_count, err_count, timeout_count,
                   response_times.copy(), error_types.copy())
        total_done = ack_count + err_count + timeout_count
        if inject_count:
            self.after(0, self._log,
                       f"스트레스 테스트 완료 (총 {total_done}건, 오류 주입 {inject_count}건)", "SYS")
        else:
            self.after(0, self._log, "스트레스 테스트 완료", "SYS")
        self.stress_running = False
        self.in_flight.release()
        self.after(0, lambda: self.btn_start_test.configure(state="normal"))
        self.after(0, lambda: self.btn_stop_test.configure(state="disabled"))

    def _update_stress_progress(self, done: int, total: int, progress: float):
        """진행률 바 업데이트 (메인 스레드에서 호출)."""
        self.progress_bar.set(progress)
        self.progress_label.configure(text=f"{done}/{total}")

    def _update_stress_stats(self, ack: int, err: int, timeout: int,
                              times: list, error_types: dict):
        """통계 행 업데이트 (메인 스레드에서 호출)."""
        # Row 1: ACK / TIMEOUT (응답속도 포함)
        received = ack + err
        if times:
            avg_ms = sum(times) / len(times)
            min_ms = min(times)
            max_ms = max(times)
            time_str = f"AVG:{avg_ms:.0f} MIN:{min_ms:.0f} MAX:{max_ms:.0f}"
        else:
            time_str = "AVG:--- MIN:--- MAX:---"
        self.stats_comm_var.set(
            f"ACK: {received}   TIMEOUT: {timeout} ({time_str})")

        # Row 2: ABLE / UNABLE + 에러 유형 분류
        r = error_types.get("RANGE", 0)
        c = error_types.get("CHARGE", 0)
        t = error_types.get("TRAVERSE", 0)
        self.stats_fire_var.set(
            f"ABLE: {ack}   UNABLE: {err} "
            f"(RANGE:{r} CHARGE:{c} TRAVERSE:{t})")

    # 종료
    def _on_close(self):
        """안전 종료: 스레드 중지, 시리얼 해제, 윈도우 파괴."""
        self.running = False
        self.stress_stop_event.set()
        self._disconnect()
        self.destroy()


# 진입점
if __name__ == "__main__":
    app = FCSClientApp()
    app.mainloop()
