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
# 是否显示每一行接收到的原始数据 (调试开关)
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

        buffer = b""

        while True:
            # 读取一行数据 (以 \n 结尾)
            # 使用 readline 可以保证 Log 不会被截断，但要求波形数据也包含换行符
            # 如果波形是纯二进制流，可能需要改用 read() 配合 buffer 解析
            line_bytes = ser.readline()

            if not line_bytes:
                continue

            try:
                # 尝试解码为字符串来判断前缀
                line_str = line_bytes.decode("utf-8", errors="ignore")

                if DEBUG_SHOW_RAW_LINE:
                    print(f"[{Fore.BLUE}DEBUG{Fore.RESET}] Got line: {repr(line_str)}")

                is_log = False
                matched_color = Fore.WHITE

                # 检查是否匹配字典中用户定义的前缀
                for prefix, color in LOG_STYLES.items():
                    if line_str.startswith(prefix):
                        is_log = True
                        matched_color = color
                        break

                if is_log:
                    # ==== 路径 1: 显示日志 ====
                    # 匹配到了字典内的前缀，上预设颜色并去掉末尾换行符以便 print 控制
                    print(f"{matched_color}{line_str.strip()}")
                else:
                    # ==== 路径 2: 转发波形/无配置颜色的普通字 ====
                    # 没有前缀匹配，直接将原始字节流转发给 VOFA+
                    forwarder.send_waveform(line_bytes)

            except Exception as e:
                print(f"{Fore.RED}[Error parsing]: {e}")

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
