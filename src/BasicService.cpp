#include "BasicService.h"
#include "tiny_framework/logging.h"
#include "version.h"
#include "yaml-cpp/yaml.h"
// [2026-01-16] 目的：将 EC-Master demo 直接嵌入 BasicService 进程（方案1：快速跑通）
// 说明：demo 作为代码库被静态链接，启动由 BasicService 负责，避免单独启动 demo 进程
#include "EcDemoApp.h"
#include "EcDemoParms.h"
#include "EcLogging.h"
#include "EcOs.h"
#include <thread>
//[2026-01-16] 目的：不再用 EcLogMsg/EcDemoLogMsg使用BasicService的日志系统，而是使用tinylog
#include <cstdarg>
#include <cstdio>

/* 供 on_terminate 使用：Ctrl+C 时置 bRun=false 并 join，避免静态析构时 std::thread 仍 joinable 导致 terminate */
static std::thread s_demo_thread;

BasicService::BasicService()
  : tiny::application()
{
}

BasicService::~BasicService()
{
    // todo
}

// [2026-01-16] 目的：将 EC-Master 的日志回调映射到 tiny_framework 日志系统
static EC_T_DWORD EC_FNCALL EcMasterLogToTiny(struct _EC_T_LOG_CONTEXT*, EC_T_DWORD, const EC_T_CHAR* fmt, ...)
{
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    // 统一用 LOG_I 输出，保证日志系统一致
    LOG_I(BasicService) << buf;
    return EC_E_NOERROR;
}

// [2026-01-16] 目的：在 BasicService 内启动 EC-Master demo（快速验证方案）
// 说明：通过构造 demo 的命令行参数（网卡/ENI/周期/时长）复用原有 demo 逻辑
static bool StartEcMasterDemo(const YAML::Node& busi_config)
{
    // [2026-01-16] 目的：没有配置时跳过 demo 启动，保证框架可空跑
    if (!busi_config["ethercat_demo"])
    {
        LOG_I(BasicService) << "ethercat_demo not configured, skip demo start";
        return true;
    }

    const auto eni_path   = busi_config["ethercat_demo"]["eni_path"].as<std::string>("");
    const auto cycle_us   = busi_config["ethercat_demo"]["cycle_us"].as<uint32_t>(1000);
    const auto duration_ms = busi_config["ethercat_demo"]["duration_ms"].as<uint32_t>(0);
    // link_layer：完整的链路层参数串，例如：
    //   sockraw 模式: "sockraw enP2p33s0"
    //   dw3504 模式:  "dw3504 2 1 custom rk3588s osdriver 0"
    const auto link_layer = busi_config["ethercat_demo"]["link_layer"].as<std::string>("");
    const bool motor_map_from_cfg = busi_config["ethercat_demo"]["motor_map_from_cfg"].as<bool>(true);

    std::string calib_path = eni_path.substr(0, eni_path.rfind('/') + 1) + "cfg.json";
    std::vector<EC_T_WORD> slave_addrs;
    std::vector<int> dds_indices;
    if (motor_map_from_cfg) {
        // 从 cfg.json 读 motor_map（只读一个文件，与 yaml 无关）
        try {
            YAML::Node cfg = YAML::LoadFile(calib_path);
            YAML::Node ma = cfg["motor_map"];
            if (ma && ma.IsSequence()) {
                if (ma.size() >= 28 && !ma[0].IsMap()) {
                    // 格式：28 个 bool 数组，true 表示该 DDS 索引为 EtherCAT 电机，站地址按顺序 1002, 1003, ...
                    EC_T_WORD next_addr = 1002;
                    for (size_t i = 0; i < ma.size() && i < 28; i++) {
                        if (ma[i].as<bool>(false)) {
                            dds_indices.push_back(static_cast<int>(i));
                            slave_addrs.push_back(next_addr++);
                        }
                    }
                } else {
                    // 旧格式：[{ "slave_addr": 1002, "dds_index": 14 }, ...]
                    for (const auto& node : ma) {
                        if (node["slave_addr"] && node["dds_index"]) {
                            slave_addrs.push_back(static_cast<EC_T_WORD>(node["slave_addr"].as<int>()));
                            dds_indices.push_back(node["dds_index"].as<int>());
                        }
                    }
                }
            }
        } catch (const std::exception&) { }
    }
    if (slave_addrs.empty() && busi_config["ethercat_demo"]["motor_map"]) {
        for (const auto& node : busi_config["ethercat_demo"]["motor_map"]) {
            slave_addrs.push_back(static_cast<EC_T_WORD>(node["slave_addr"].as<int>()));
            dds_indices.push_back(node["dds_index"].as<int>());
        }
    }
    if (!slave_addrs.empty()) {
        EcDemo_SetMotorConfig(slave_addrs.data(), dds_indices.data(),
                              static_cast<int>(slave_addrs.size()),
                              calib_path.c_str());
        LOG_I(BasicService) << "motor_map: " << slave_addrs.size() << " motors"
            << (motor_map_from_cfg ? " (from cfg.json)" : " (from busi.yaml)") << ", calib: " << calib_path;
    }

    // [2026-01-16] 目的：关键参数缺失时直接报错，避免 demo 进入异常状态
    if (link_layer.empty() || eni_path.empty())
    {
        LOG_COUT(BasicService) << "ethercat_demo.link_layer or eni_path empty";
        return false;
    }

    // [2026-01-16] 目的：demo 在独立线程运行，避免阻塞 BasicService 初始化流程
    if (s_demo_thread.joinable())
    {
        LOG_I(BasicService) << "ecmaster demo already running";
        return true;
    }

    s_demo_thread = std::thread([link_layer, eni_path, cycle_us, duration_ms]() {
        // [2026-01-16] 目的：最小化复用 EcDemoMain.cpp 的上下文初始化流程
        T_EC_DEMO_APP_CONTEXT AppContext;
        OsMemset(&AppContext, 0, sizeof(AppContext));

        // [2026-01-16] 目的：设置 demo 的运行标志，驱动主循环进入工作状态
        bRun = EC_TRUE;

        // [2026-01-16] 目的：初始化 demo 日志（输出到 stdout）
        AppContext.LogParms.dwLogLevel = EC_LOG_LEVEL_INFO;
        AppContext.LogParms.pfLogMsg = EcMasterLogToTiny;
        AppContext.LogParms.pLogContext = EC_NULL;
        OsMemcpy(G_pEcLogParms, &AppContext.LogParms, sizeof(EC_T_LOG_PARMS));

        AppContext.dwInstanceId = INSTANCE_MASTER_DEFAULT;
        ResetAppParms(&AppContext, &AppContext.AppParms);

        AppContext.AppParms.Os.dwSize = sizeof(EC_T_OS_PARMS);
        AppContext.AppParms.Os.dwSignature = EC_OS_PARMS_SIGNATURE;
        AppContext.AppParms.Os.dwSupportedFeatures = 0xFFFFFFFF;
        AppContext.AppParms.Os.PlatformParms.bConfigMutex = EC_TRUE;
        AppContext.AppParms.Os.PlatformParms.nMutexType = PTHREAD_MUTEX_RECURSIVE;
        AppContext.AppParms.Os.PlatformParms.nMutexProtocol = PTHREAD_PRIO_NONE;
        OsInit(&AppContext.AppParms.Os);

        // [2026-01-16] 目的：构造 demo 命令行参数，复用 demo 的参数解析逻辑
        EC_T_CHAR szCmd[COMMAND_LINE_BUFFER_LENGTH];
        OsMemset(szCmd, 0, sizeof(szCmd));
        if (duration_ms > 0)
        {
            OsSnprintf(szCmd, sizeof(szCmd) - 1, "-%s -f \"%s\" -b %u -t %u",
                       link_layer.c_str(), eni_path.c_str(), cycle_us, duration_ms);
        }
        else
        {
            OsSnprintf(szCmd, sizeof(szCmd) - 1, "-%s -f \"%s\" -b %u",
                       link_layer.c_str(), eni_path.c_str(), cycle_us);
        }

        // [2026-01-16] 目的：将命令行参数写入 demo 参数结构体
        if (EC_E_NOERROR != SetAppParmsFromCommandLine(&AppContext, szCmd, &AppContext.AppParms))
        {
            LOG_COUT(BasicService) << "SetAppParmsFromCommandLine failed";
            return;
        }

        // [2026-01-16] 目的：正式运行 demo 主流程（主站初始化、进入 OP、周期任务）
        (void)EcDemoApp(&AppContext);

        // [2026-01-16] 目的：释放 demo 运行过程中分配的参数资源
        FreeAppParms(&AppContext, &AppContext.AppParms);
    });

    LOG_I(BasicService) << "ecmaster demo thread started";
    return true;
}

bool BasicService::initialize(const std::string& busi_config)
{
    if (!init_config(busi_config))
    {
        return false;
    }

    // [2026-01-16] 目的：在 BasicService 初始化阶段拉起 demo（快速集成验证）
    try
    {
        YAML::Node busi = YAML::LoadFile(busi_config);
        if (!StartEcMasterDemo(busi))
        {
            return false;
        }
    }
    catch (const std::exception& e)
    {
        LOG_COUT(BasicService) << "ethercat_demo config load failed: " << e.what();
        return false;
    }

    // 【注意】需要app.yaml中daemon设置为true时，添加定时器才生效
    // 这里count需要声明为static，否则post_timer_event结束后，函数退出，count因退出作用域而会被销毁
    static uint32_t count = 0;
    event_loop_.post_timer_event(
    "timer_test",
    [&]() { LOG_I(BasicService) << "timer_test, count: " << ++count; },
    std::chrono::seconds(1),
    5);

    LOG_I(BasicService) << app_name() << "_" << app_version() << "(" << app_buildtime()
                   << ") already initialized.";
    return true;
}

bool BasicService::init_config(const std::string& busi_config)
{
    if (busi_config.empty())
    {
        LOG_COUT(BasicService) << "busi_config is empty";
    }

    try
    {
        // 加载配置文件
        YAML::Node config = YAML::LoadFile(busi_config);

        // 解析单进程配置
        auto test_property = config["test_property"].as<std::string>();
        LOG_I(BasicService) << LOG_KV(test_property);

        return true;
    }
    catch (const YAML::Exception& e)
    {
        TINYLOG_STDERR(BasicService) << "parsing " << busi_config << " YAML: " << e.what();
        return false;
    }
    catch (const std::exception& e)
    {
        TINYLOG_STDERR(BasicService) << "Error: " << e.what();
        return false;
    }
}

void BasicService::on_terminate()
{
    LOG_COUT(BasicService) << "on_terminate";
    /* 通知 EcDemoApp 主循环退出，并 join demo 线程，避免进程退出时 std::thread 析构调用 terminate */
    bRun = EC_FALSE;
    if (s_demo_thread.joinable())
    {
        s_demo_thread.join();
    }
}

std::string BasicService::app_version() const
{
    return APP_VERSION;
}

std::string BasicService::app_buildtime() const
{
    return APP_BUILD_TIME;
}

std::string BasicService::app_name() const
{
    return APP_NAME;
}
