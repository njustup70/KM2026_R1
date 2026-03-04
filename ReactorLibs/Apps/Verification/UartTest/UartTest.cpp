#include "UartTest.hpp"
#include "bsp_log.hpp"

#include <cstring>

static void OnPortRx(const uint8_t *payload, uint8_t len)
{
    // 如果想要看接收，可以自行用调试器或抓包工具看。
    // 由于可能收包也会很快，避免在这里打印BspLog。
    // 但是为了最基本的确认，如果收到特定字符或者特定包，可以进行状态闪烁等等，这里先只记一句日志。
    // BspLog_LogInfo("[UartTest] Port 0x10 Rx %d bytes.", len);
}

void UartTest::Start()
{
    bool ok = portor_.Init(Hardware::huart_host, 0x10, OnPortRx);
    
    // 填充测试用的发送缓冲区：填充为 'A' ~ 'A'+63 模拟有规律的数据
    for(int i = 0; i < 63; i++) 
    {
        test_pkg_[i] = 'A' + (i % 26);
    }
    test_pkg_[63] = '\n';
    
    if(ok) 
    {
        BspLog_LogInfo("UartTest Started! Portor init OK (h_other).");
    } else 
    {
        BspLog_LogError("UartTest Start Failed! Portor Init failed.");
    }
    
    tick_cnt_ = 0;
    stage_ = TestStage::PREPARE;
}

void UartTest::Update()
{
    tick_cnt_++;

    switch (stage_)
    {
    case TestStage::PREPARE:
        // 挂机 1 秒 (200帧) 后开始
        if (tick_cnt_ >= 200) {
            stage_ = TestStage::BANDWIDTH_TEST;
            tick_cnt_ = 0;
            BspLog_LogInfo("=== [Stage 1] Bandwidth Test Started ===");
        }
        break;

    case TestStage::BANDWIDTH_TEST:
        // 在 10 秒内 (2000帧) 进行满功率测试
        if (tick_cnt_ < 2000) {
            portor_.Send(test_pkg_, 64);
        } else {
            stage_ = TestStage::STAGE_GAP;
            tick_cnt_ = 0;
            BspLog_LogInfo("\r\n=== [Stage 1] Bandwidth Test Finished ===");
            BspLog_LogInfo("=== Stage Gap: 1s quiet time before Stage 2 ===");
        }
        break;

    case TestStage::STAGE_GAP:
        // 阶段间隔：静默 1 秒 (200帧)，避免上位机日志紧贴不易分辨
        if (tick_cnt_ >= 200) {
            stage_ = TestStage::FIFO_TEST;
            tick_cnt_ = 0;
            BspLog_LogInfo("\r\n=== [Stage 2] FIFO Burst Test Started ===");
        }
        break;

    case TestStage::FIFO_TEST:
        // 在接下来的 2 秒内 (400帧) 压测 FIFO 瞬发接纳力
        if (tick_cnt_ < 400) {
            // 每 10 帧 (50ms) 触发一次突发
            if (tick_cnt_ % 10 == 0) {
                // 瞬间灌入 15 个包 (15 * (64 + 信封) 约 1050 字节，没超过 2048 的 FIFO 上限)
                for (int i = 0; i < 15; i++) {
                    portor_.Send(test_pkg_, 64);
                }
            }
        } else {
            stage_ = TestStage::DONE;
            tick_cnt_ = 0;
            BspLog_LogOK("UartTest Stages Completed. Idling.");
        }
        break;

    case TestStage::DONE:
        // 测试完成，保持静默即可
        // 可以每隔 5 秒报下平安
        if(tick_cnt_ >= 1000) {
            BspLog_LogInfo("UartTest is done and sleeping.");
            tick_cnt_ = 0;
        }
        break;
    }
}
