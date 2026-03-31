import subprocess
import sys
import ctypes
from pathlib import Path


REACTOR_CMAKE_INCLUDE = "include(ReactorLibs/Reactor46H.cmake)"
STDCPP_INCLUDE = '#include "std_cpp.h"'
INIT_CALL = "  Reactor46H_Initialize();"
EXTENSIONS_REPO_URL = "https://github.com/njustup70/Extensions70.git"
COLOR_GREEN = "\033[32m"
COLOR_RED = "\033[31m"
COLOR_BLUE = "\033[34m"
COLOR_WHITE = "\033[37m"
COLOR_RESET = "\033[0m"


def enable_ansi_colors() -> None:
    if sys.platform != "win32":
        return
    try:
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetStdHandle(-11)  # STD_OUTPUT_HANDLE
        mode = ctypes.c_uint32()
        if kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
            kernel32.SetConsoleMode(handle, mode.value | 0x0004)
    except Exception:
        pass


def color_print(message: str, color: str) -> None:
    if sys.stdout.isatty():
        print(f"{color}{message}{COLOR_RESET}")
    else:
        print(message)


def log_success(message: str) -> None:
    color_print(message, COLOR_GREEN)


def log_fail(message: str) -> None:
    color_print(message, COLOR_RED)


def log_no_change(message: str) -> None:
    color_print(message, COLOR_WHITE)


def log_info(message: str) -> None:
    color_print(message, COLOR_BLUE)


def detect_newline(text: str) -> str:
    return "\r\n" if "\r\n" in text else "\n"


def read_text(path: Path) -> str:
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")
    return path.read_text(encoding="utf-8")


def write_text(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8", newline="")


def patch_cmake(root: Path) -> bool:
    cmake_path = root / "CMakeLists.txt"
    text = read_text(cmake_path)
    newline = detect_newline(text)

    lines = text.splitlines()
    last_non_empty = ""
    for line in reversed(lines):
        if line.strip():
            last_non_empty = line.strip()
            break

    if last_non_empty == REACTOR_CMAKE_INCLUDE:
        log_no_change("[CMakeLists] no change needed")
        return False

    patched = text.rstrip() + newline + REACTOR_CMAKE_INCLUDE + newline
    write_text(cmake_path, patched)
    log_success("[CMakeLists] appended Reactor46H include")
    return True


def find_region(lines: list[str], begin_marker: str, end_marker: str) -> tuple[int, int]:
    begin_idx = next((i for i, line in enumerate(lines) if begin_marker in line), -1)
    if begin_idx == -1:
        raise RuntimeError(f"Missing marker: {begin_marker}")

    end_idx = next(
        (i for i in range(begin_idx + 1, len(lines)) if end_marker in lines[i]),
        -1,
    )
    if end_idx == -1:
        raise RuntimeError(f"Missing marker: {end_marker}")

    return begin_idx, end_idx


def ensure_line_in_region(
    lines: list[str], begin_marker: str, end_marker: str, expected_line: str, newline: str
) -> tuple[list[str], bool]:
    begin_idx, end_idx = find_region(lines, begin_marker, end_marker)
    region = lines[begin_idx + 1 : end_idx]
    if any(item.strip() == expected_line.strip() for item in region):
        return lines, False

    lines.insert(end_idx, expected_line + newline)
    return lines, True


def patch_main(root: Path) -> bool:
    main_path = root / "Core" / "Src" / "main.c"
    text = read_text(main_path)
    newline = detect_newline(text)
    lines = text.splitlines(keepends=True)

    changed = False
    lines, include_changed = ensure_line_in_region(
        lines,
        "/* USER CODE BEGIN Includes */",
        "/* USER CODE END Includes */",
        STDCPP_INCLUDE,
        newline,
    )
    changed = changed or include_changed

    lines, init_changed = ensure_line_in_region(
        lines,
        "/* USER CODE BEGIN 2 */",
        "/* USER CODE END 2 */",
        INIT_CALL,
        newline,
    )
    changed = changed or init_changed

    if changed:
        write_text(main_path, "".join(lines))
        log_success("[main.c] patched USER CODE sections")
    else:
        log_no_change("[main.c] no change needed")

    return changed


def run_sync_keil(root: Path) -> None:
    sync_script = root / "sync_keil.py"
    if not sync_script.exists():
        raise FileNotFoundError(f"sync_keil.py not found: {sync_script}")

    log_info("[sync_keil] running")
    subprocess.run(
        [sys.executable, str(sync_script)],
        cwd=str(root),
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    log_success("[sync_keil] completed")


def pull_extensions_repo(root: Path) -> None:
    reactor_lib_dir = root / "ReactorLibs"
    extensions_dir = reactor_lib_dir / "Extensions70"

    reactor_lib_dir.mkdir(parents=True, exist_ok=True)

    if extensions_dir.exists():
        log_no_change("[Exten] repo already exists")
        return

    try:
        result = subprocess.run(
            ["git", "clone", "--quiet", EXTENSIONS_REPO_URL, str(extensions_dir)],
            cwd=str(root),
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            log_success("[Exten] got Extensions70 repo")
        else:
            log_fail("[Exten] unable to fetch")
    except Exception:
        log_fail("[Exten] unable to fetch")


def main() -> int:
    enable_ansi_colors()
    root = Path(__file__).resolve().parent
    log_no_change("---------------------------------------")
    log_info(f"[框架构建] 路径 : {root}")
    try:
        patch_cmake(root)
        patch_main(root)
        pull_extensions_repo(root)
        run_sync_keil(root)
    except Exception as exc:
        log_fail(f"[框架构建] 发生错误: {exc}")
        return 1

    log_success("[框架构建] 构建完成")
    log_no_change("---------------------------------------")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
