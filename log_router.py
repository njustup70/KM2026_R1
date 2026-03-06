import serial
import serial.tools.list_ports
import socket
import threading
import sys
import select
import time
from colorama import init, Fore, Style

# 初始化 Colorama 以支持 Windows 终端颜色
init(autoreset=True)

# ================= 配置区域 =================
# 是否显示每一行接收到的原始数据 (调试开关)，可通过终端动态配置
DEBUG_SHOW_RAW_LINE = False

# 转发给 VOFA+ 的 TCP 端口 (VOFA+ 选 TCP Client 连接此端口)
TCP_HOST = "0.0.0.0"
TCP_PORT = 8888

# 根据 bsp_log.cpp 定义的日志前缀与颜色映射
# 只有匹配到以下前缀的文本，才被认为是日志在终端截留着色，否则一律当作波形转给 VOFA+
# 您可以在这里面自定义需要截获的前缀和您期望设定的颜色
LOG_STYLES = {
    "[Error] ": Fore.RED + Style.BRIGHT,
    "[Warn] ": Fore.YELLOW + Style.BRIGHT,
    "[Well] ": Fore.GREEN + Style.BRIGHT,
    "[Note] ": Fore.MAGENTA + Style.BRIGHT,  # 对应 STM32 的 Purple
    "[Respond] ": Fore.CYAN + Style.BRIGHT,  # 对应 STM32 的 Blue
    "[Info] ": Fore.WHITE + Style.BRIGHT,  # 例子：添加对 Info 的白字拦截
}

# ===========================================


class DataForwarder:
    def __init__(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_socket = None
        self.running = True

    def start_server(self):
        """启动 TCP 服务器等待 VOFA+ 连接"""
        try:
            self.server_socket.bind((TCP_HOST, TCP_PORT))
            self.server_socket.listen(1)
            print(
                f"{Fore.CYAN}[System] TCP Server listening on port {TCP_PORT}. Connect VOFA+ via TCP Client mode."
            )
        except Exception as e:
            print(f"{Fore.RED}[System] Failed to bind TCP port: {e}")
            return

        while self.running:
            try:
                # 阻塞等待连接
                client, addr = self.server_socket.accept()
                print(f"{Fore.GREEN}[System] VOFA+ Connected from {addr}")
                self.client_socket = client

                # 保持连接直到断开
                while self.running:
                    try:
                        # select(read_list, write_list, error_list, timeout)
                        # timeout=0 表示非阻塞，立即返回结果
                        # r 列表如果不为空，说明 client_socket 有数据（或断开信号）来了
                        r, _, _ = select.select([self.client_socket], [], [], 0)

                        if r:
                            # 只有当 select 说“有动静”时，才去 peek 数据
                            # 这样就不需要 MSG_DONTWAIT 标志了
                            data = self.client_socket.recv(16, socket.MSG_PEEK)
                            if data == b"":
                                # 读到空数据意味着对端（VOFA+）关闭了连接
                                raise ConnectionResetError
                    except (ConnectionResetError, OSError):
                        break
                    time.sleep(0.5)

                print(f"{Fore.YELLOW}[System] VOFA+ Disconnected")
                if self.client_socket:
                    self.client_socket.close()
                    self.client_socket = None
            except Exception as e:
                if self.running:
                    print(f"{Fore.RED}[System] TCP Error: {e}")

    def send_waveform(self, data_bytes):
        """转发数据给 VOFA+"""
        if DEBUG_SHOW_RAW_LINE:
            print(f"[{Fore.BLUE}DEBUG-WAVE{Fore.RESET}] Forwarding: {repr(data_bytes)}")

        if self.client_socket:
            try:
                self.client_socket.sendall(data_bytes)
            except:
                # 发送失败通常意味着连接断开了，主循环会处理
                pass

    def stop(self):
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        self.server_socket.close()


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]


def main():
    # 1. 选择串口
    ports = list_serial_ports()
    if not ports:
        print(f"{Fore.RED}No serial ports found!")
        return

    print("Available Ports:")
    for i, p in enumerate(ports):
        print(f"{i}: {p}")

    try:
        idx = int(input("Select Port Index: "))
        port_name = ports[idx]
        baud_rate = int(input("Baudrate: ") or "115200")

        debug_input = input("Show raw/debug binary data? (y/N): ").strip().lower()
        if debug_input == "y":
            global DEBUG_SHOW_RAW_LINE
            DEBUG_SHOW_RAW_LINE = True
        else:
            DEBUG_SHOW_RAW_LINE = False

    except:
        print("Invalid selection.")
        return

    # 启动 TCP 转发线程
    forwarder = DataForwarder()
    tcp_thread = threading.Thread(target=forwarder.start_server, daemon=True)
    tcp_thread.start()

    # 打开串口并开始处理
    try:
        ser = serial.Serial(port_name, baud_rate, timeout=0.1)
        print(f"{Fore.GREEN}[System] Opened {port_name} @ {baud_rate}")
        print(
            f"{Fore.GREEN}[System] Logs will appear here. Waveforms forwarded to TCP :{TCP_PORT}"
        )
        print("-" * 40)

        buffer = bytearray()
        # 预先将所有的前缀转换为 bytes，以优化判断速度
        prefixes_bytes = [p.encode("utf-8") for p in LOG_STYLES.keys()]
        # 提取所有前缀的首字节，以便在纯二进制流中快速定位下一个可能的日志起始位置
        prefix_first_bytes = list(set([bytes([p[0]]) for p in prefixes_bytes]))

        def could_be_log(buf):
            # 判断 buf 是否可能是一条彩色日志的开头
            for p in prefixes_bytes:
                # 只要 buf 和 p 在可比较的长度内完全一致，就有可能是日志
                match_len = min(len(p), len(buf))
                if p[:match_len] == buf[:match_len]:
                    return True
            return False

        while True:
            # 使用 read 获取可用数据，避免 readline 在无 '\n' 的纯二进制流上阻塞
            # in_waiting 是当前串口缓冲区已接收的字节数
            data = ser.read(max(1, ser.in_waiting))
            if not data:
                continue

            buffer.extend(data)

            while buffer:
                if could_be_log(buffer):
                    # 如果匹配了日志的前缀特征，则尝试寻找 '\n' 来截取完整的一行
                    nl_idx = buffer.find(b"\n")
                    if nl_idx != -1:
                        # 找到了换行符，提取出一整行
                        line_bytes = bytes(buffer[: nl_idx + 1])
                        del buffer[: nl_idx + 1]

                        try:
                            line_str = line_bytes.decode("utf-8", errors="ignore")
                            is_log = False
                            matched_color = Fore.WHITE

                            for prefix, color in LOG_STYLES.items():
                                if line_str.startswith(prefix):
                                    is_log = True
                                    matched_color = color
                                    break

                            if DEBUG_SHOW_RAW_LINE:
                                print(
                                    f"[{Fore.BLUE}DEBUG{Fore.RESET}] Got line: {repr(line_str)}"
                                )

                            if is_log:
                                # ==== 路径 1: 显示彩色日志 ====
                                print(f"{matched_color}{line_str.strip()}")
                            else:
                                # ==== 路径 2: 转发未匹配特征的数据 ====
                                forwarder.send_waveform(line_bytes)
                        except Exception as e:
                            print(f"{Fore.RED}[Error parsing]: {e}")
                    else:
                        # 没有找到换行符，但前缀有可能合法，等待后续数据
                        # 避免异常长数据（如含前缀但无换行符的二进制）无限堆积内存，设个2048字节阈值
                        if len(buffer) > 2048:
                            forwarder.send_waveform(bytes(buffer[:1]))
                            del buffer[:1]
                        else:
                            break  # 跳出内层缓冲区处理，继续从串口读取新数据
                else:
                    # 头字符不符合任何预设的 Log 前缀，说明属于纯二进制波形或普通字符
                    # 寻找任何一个合法前缀的首字符，作为可能的新 Log 的起始点
                    next_starts = [buffer.find(b, 1) for b in prefix_first_bytes]
                    valid_starts = [s for s in next_starts if s != -1]

                    if valid_starts:
                        next_start = min(valid_starts)
                        # 截止到下个潜在日志起点前的数据，全数丢给波形转发（解决粘包且低延迟）
                        forwarder.send_waveform(bytes(buffer[:next_start]))
                        del buffer[:next_start]
                    else:
                        # 整个缓冲区都没有潜在日志起点，全量直接转发
                        forwarder.send_waveform(bytes(buffer))
                        buffer.clear()

    except serial.SerialException as e:
        print(f"{Fore.RED}Serial Error: {e}")
    except KeyboardInterrupt:
        print(f"\n{Fore.YELLOW}Stopping...")
    finally:
        forwarder.stop()
        if "ser" in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
