# 高频命令导致 std::bad_alloc 崩溃问题分析与零堆分配改造

本文档记录一次真实的固件崩溃事故：上位机以 50Hz 下发运动命令时固件报
`std::bad_alloc` 后死亡，降到 10Hz 则正常。文中会先科普必要的内存管理知识，
再按 **问题现象 → 原设计思路 → 问题根源 → 新设计思路** 的顺序完整复盘，
所有结论都对应到具体源码位置。

---

## 1. 问题描述

### 1.1 现象

上位机按 **50Hz** 频率持续通过 USB 下发形如：

```text
>{j1},{j2},{j3},{j4},{j5},{j6}\n
```

的关节运动命令。固件工作在 `COMMAND_TARGET_POINT_INTERRUPTABLE`（可打断模式），
走 `ParseCommand()` 中 L495 起的分支。运行一段时间后，上位机收到的回复变成：

```text
<< context->MoveJ succeeded
<< ok
<< terminate called after throwing an instance of 'std::bad_alloc'
<< what():  std::bad_alloc
```

之后固件**彻底失去响应**，只能复位。

把下发频率降到 **10Hz**，长时间运行不再复现。

### 1.2 两个关键疑问

1. `terminate called after throwing...` 这两行是 C++ 运行时的崩溃信息，
   为什么会出现在上位机的接收窗口里？
2. 为什么频率从 50Hz 降到 10Hz 问题就消失了？是不是单纯"发太快"？

先回答第一个：**这两行确实是固件发出来的**。固件的 `_write()`（printf/cerr
最终都会调用它）把所有输出同时转发到 USB 和 UART4：

```cpp
// Bsp/communication/communication.cpp
int _write(int file, const char* data, int len)
{
    usbStreamOutputPtr->process_bytes((const uint8_t*) data, len, nullptr);
    uart4StreamOutputPtr->process_bytes((const uint8_t*) data, len, nullptr);
    return len;
}
```

C++ 程序抛出未捕获的 `std::bad_alloc` 异常时会调用 `terminate()`，
它把报错写到 stderr → 经 `_write()` → 出现在 USB 输出里 → 上位机收到。
随后固件 `abort()`，所以不再响应任何命令。

**结论：崩溃发生在下位机固件内部，与上位机软件无关。**

第二个疑问的答案见第 4 节——这不是"速度太快"，而是一个**概率随频率升高
而增大的多线程内存竞争 bug**。

---

## 2. 背景知识：内存管理科普（小白必读）

要看懂这个 bug，需要理解下面几个概念。尽量用类比讲清楚。

### 2.1 程序的三块内存：栈、堆、静态区

一个 C/C++ 程序的内存大致分三块：

| 区域 | 谁管理 | 特点 | 类比 |
|---|---|---|---|
| **栈（Stack）** | 编译器自动管理 | 函数进出的局部变量，进出极快，函数结束自动回收，大小有限 | 你自己桌上的托盘，随用随放、用完即收 |
| **堆（Heap）** | 程序员手动管理（`malloc/free`、`new/delete`） | 想要多大申请多大，但必须自己记得归还，速度慢、规则复杂 | 公共仓库，要填单申请货架（malloc），用完必须填单归还（free） |
| **静态区（.data/.bss）** | 编译器 | 全局变量、static 变量，程序启动到结束一直存在 | 单位的固定办公室 |

关键点：**栈上的操作不需要任何"协商"，天然安全且快；堆上的操作要走一套
管理流程，流程一旦被破坏就会出事。**

### 2.2 malloc/free 是怎么工作的

堆在 STM32 上就是 RAM 里划出来的一段空间（本工程由链接脚本
`STM32F405RGTx_FLASH.ld` 规定：`_Min_Heap_Size = 0x3C00`，即 **15KB**）。

`malloc` 内部维护一张"空闲货架清单"（free list）：

```text
malloc(40):  在空闲清单里找一块 ≥40 字节的空隙 → 记一笔账 → 返回指针
free(ptr):   把这块空间挂回空闲清单 → 以后可以被复用
```

这张清单本身也存在内存里。**如果两个使用者同一刻都在改这张清单，
而中间没有任何锁，清单就可能被改坏**——比如同一块空间被挂上去两次、
指针被改成垃圾值。清单一旦坏了，之后的 `malloc` 要么失败返回 NULL，
要么发给你一块和别处重叠的内存，程序行为彻底失控。这叫**堆损坏
（heap corruption）**，是嵌入式里最阴险的 bug 之一，因为它往往
"损坏发生在 A 时刻，爆炸发生在 B 时刻"。

### 2.3 std::string 为什么和堆有关

`std::string` 是 C++ 的字符串类。它的**内容存在堆上**：

```cpp
std::string s = "hello";   // 内部偷偷执行了 malloc，把 "hello" 拷到堆上
// ...
// s 生命周期结束时，内部偷偷执行 free
```

也就是说，**写代码时看不到 malloc，但只要用了 std::string，
每次构造/析构都在悄悄走堆**。这是很多 PC 思维写嵌入式代码时踩的坑：
PC 内存以 GB 计、操作系统帮你兜底；MCU 只有 15KB 堆，且没有任何兜底。

另一个隐式陷阱：**函数参数类型不匹配时的隐式转换**。

```cpp
void Push(const std::string &cmd);   // 形参是 std::string

const char* cmd = ">10,20,60,0,0,0";
Push(cmd);   // 编译器"好心"地帮你先构造一个临时 std::string
             // = 偷偷 malloc 一次，调用结束后偷偷 free 一次
```

表面上只是传了个字符串，实际每条命令都发生了堆分配。**原设计正是栽在这里。**

### 2.4 多线程下的"共享设施"必须加锁

FreeRTOS 里多个线程是"真的同时在跑"（在时间片上交替，且会被中断打断）。
任何被多个线程共享的数据结构，访问时都必须加锁（互斥量/临界区），否则
两个线程的操作会互相踩踏，术语叫**竞态条件（race condition）**。

一个形象的例子：公共仓库只有一本登记簿。
- 线程 A 正在"归还 3 号货架"，刚在簿子上写了一半；
- 线程 B 插进来"申请货架"，读到了写了一半的簿子；
- 簿子从此记录错乱，之后谁申请都可能拿到错误结果。

解决办法只有一个：**用登记簿之前先抢一把钥匙（锁），用完还回去。**

### 2.5 newlib 的 malloc 靠谁来加锁？

arm-none-eabi 工具链的 C 库叫 **newlib**（本工程还用了精简版
`nano.specs`）。newlib 的设计是"可移植"的：它的 `malloc/free` 内部会调用

```c
__retarget_lock_acquire(__malloc_lock);    // 加锁
__retarget_lock_release(__malloc_lock);    // 解锁
```

但这两个函数**留给使用者自己实现**（因为不同系统的锁实现不同）。
如果工程里没实现，链接器会用 libgloss 提供的**空实现（weak 符号）**——
等于**什么锁都没加**。

> 本工程（改造前）恰好就没有实现它们 → 在 FreeRTOS 多线程环境下，
> **newlib 的 malloc/free 是完全不加锁的**。

顺带一提：FreeRTOS 自己那套内存接口（`pvPortMalloc`，配了 64KB 的
heap_4 堆）内部是自带临界区保护的，但它只管 RTOS 内部对象
（任务栈、队列等）；`std::string`/`operator new` 走的是 newlib 那套
15KB 的堆，两者互不相干。

### 2.6 std::bad_alloc 是什么

C++ 里 `new` 申请内存失败（底层 `malloc` 返回 NULL）时，会抛出
`std::bad_alloc` 异常。如果没有任何 `try/catch` 接住它，程序就调用
`terminate()` 打印那句 `terminate called after throwing...` 然后 `abort()`。

---

## 3. 原来的设计思路

### 3.1 命令流水线长什么样

固件用一条 **FreeRTOS 消息队列（FIFO）** 把"收命令"和"执行命令"解耦：

```text
USB 收包线程 (UsbServerTask)          命令执行线程 (ThreadControlLoopUpdate)
─────────────────────────────        ──────────────────────────────────────
ASCII_protocol_parse_stream()
  → OnUsbAsciiCmd()
      → commandHandler.Push(_cmd)  ──────►  commandFifo (16 条 × 64 字节)
      → Respond("ok queued free=N")                 │
                                                    ▼
                                    commandHandler.Pop(osWaitForever)
                                      → commandHandler.ParseCommand(cmd)
                                          → MoveJ() / MoveJoints() ...
```

这套设计本身是**合理且经典的**：

- 收包线程只做"入队 + 立即应答"，绝不被运动执行阻塞，USB 收发不会卡；
- 执行线程专心消费队列，突发多条命令时自动排队，天然起到缓冲削峰作用；
- 两个线程只通过队列交换数据，表面上职责清晰。

它明显脱胎于 ODrive 风格的代码，用 C++ 的 `std::string` 传递命令，
写法和 PC 软件一模一样——**问题恰恰出在这个"PC 式写法"上**。

### 3.2 原实现代码（改造前）

```cpp
// Robot/instances/dummy_robot.h
uint32_t Push(const std::string &_cmd);       // 形参是 std::string
std::string Pop(uint32_t timeout);            // 返回值是 std::string
uint32_t ParseCommand(const std::string &_cmd);

// Robot/instances/dummy_robot.cpp
uint32_t DummyRobot::CommandHandler::Push(const std::string &_cmd)
{
    osStatus_t status = osMessageQueuePut(commandFifo, _cmd.c_str(), 0U, 0U);
    ...
}

std::string DummyRobot::CommandHandler::Pop(uint32_t timeout)
{
    osStatus_t status = osMessageQueueGet(commandFifo, strBuffer, nullptr, timeout);
    return std::string{strBuffer};            // 每次 Pop 都构造一个 std::string
}

// UserApp/main.cpp —— 执行线程
for (;;)
{
    dummy.commandHandler.ParseCommand(dummy.commandHandler.Pop(osWaitForever));
}

// UserApp/protocols/ascii_protocol.cpp —— USB 收包线程
uint32_t freeSize = dummy.commandHandler.Push(_cmd);   // _cmd 是 const char*
```

---

## 4. 原设计存在的问题

### 4.1 主病灶：每条命令两次堆分配，且分布在两个线程

逐条命令数一数堆操作：

| 步骤 | 所在线程 | 堆操作 |
|---|---|---|
| ① `Push(_cmd)`：`const char*` 隐式转换成临时 `std::string` | USB 收包线程 | `malloc` + `free` |
| ② `Pop()` 返回 `std::string{strBuffer}` | 命令执行线程 | `malloc` + `free` |

**每条命令 = 两个线程、共 2 对 malloc/free。**
50Hz 下发 = 每秒 100 对并发堆操作。

结合第 2.5 节的事实——本工程 newlib 的 malloc/free **没有加锁**——
两个线程随时可能同时挤进 malloc/free 内部改同一张空闲清单：

```text
USB线程:     malloc ─────────┐
执行线程:              free ─┼──── 同一瞬间操作同一张 free list
                             ▼
                     空闲清单被写坏（堆损坏）
                             ▼
              之后某次 malloc 失败，返回 NULL
                             ▼
             operator new 抛出 std::bad_alloc（无人捕获）
                             ▼
        terminate() → stderr 报错经 _write() 发到 USB → abort()
```

### 4.2 为什么 50Hz 崩、10Hz 不崩？

这是**概率问题，不是阈值问题**：

- 竞争只在"两个线程同时处于 malloc/free 内部"的极短窗口内发生；
- 50Hz 时每秒有 100 对并发堆操作，撞上窗口的期望时间很短，
  堆损坏几秒到几分钟内必然发生；
- 10Hz 时并发频率降为 1/5，测试时长内恰好没触发——**不代表安全，
  只是雷被推迟了**。

可以顺手排除两种"看起来像"的原因：

- **不是队列被撑爆**：`commandFifo` 是 16×64 的队列，可打断模式下
  `ParseCommand` 不等待运动完成、立即返回，消费跟得上；即使真满了，
  也只会回 `"error queue full"`，不会抛异常。
- **不是内存被耗尽（泄漏）**：这些 `std::string` 每次都是配对
  分配/释放的，稳态下堆占用不增长。会随频率恶化而崩溃的，
  只有竞态型损坏。

### 4.3 附带隐患：Push 的 64 字节越界读

队列消息大小固定为 64 字节，`osMessageQueuePut` 会从源地址**固定拷贝
64 字节**。原代码直接传 `_cmd.c_str()`，而命令只有约 40 字节——
相当于每次都越界读取字符串缓冲区后面的相邻堆内存入队。
之前侥幸没炸，只是因为前 64 字节内总能碰到 `\0`，解析不受影响，
但这属于未定义行为，也是改造中一并修掉的点。

### 4.4 次要问题：所有线程共享同一个 _reent

`FreeRTOSConfig.h` 没有定义 `configUSE_NEWLIB_REENTRANT`（默认 0），
意味着 newlib 的可重入结构 `_reent` 是全局唯一、所有线程共享的。
多线程同时调用 `sscanf("%f")`、`snprintf` 等也各自存在数据竞争，
属于同一类"newlib + RTOS 没配置好"的问题，只是本次的崩溃主因是 malloc。

---

## 5. 改后的设计思路：整条命令链路零堆分配

### 5.1 核心思想

既然病灶是"每条命令都要走两把没锁的堆"，最彻底的修法不是给堆加锁，
而是**让命令链路一次堆都不走**：全程只用固定大小的 `char[64]`
（栈上或静态缓冲区），命令进队、出队、解析都只是内存拷贝。

原则可以概括为一句话：

> **在嵌入式的高频/中断路径上，用定长缓冲区替代一切 std::string 和 new。**

### 5.2 接口变化

```cpp
// Robot/instances/dummy_robot.h
class CommandHandler
{
public:
    // Fixed message size of the FIFO, avoids any per-command heap allocation
    static const uint32_t CMD_MAX_LENGTH = 64;

    explicit CommandHandler(DummyRobot* _context) : context(_context)
    {
        commandFifo = osMessageQueueNew(16, CMD_MAX_LENGTH, nullptr);
    }

    uint32_t Push(const char* _cmd);              // 收 const char*，不再有隐式转换
    bool Pop(char* _buffer, uint32_t timeout);    // 填充调用方缓冲区，不返回 std::string
    uint32_t ParseCommand(const char* _cmd);      // 解析也改为 const char*
    ...
};
```

三个要点：

1. 把队列消息尺寸 64 提取成常量 `CMD_MAX_LENGTH`，
   队列尺寸和缓冲区尺寸从此不可能脱节；
2. 删掉原来的成员 `char strBuffer[64]`，出队缓冲区由调用方提供；
3. 所有接口回归 C 字符串，`std::string` 从命令链路中彻底消失。

### 5.3 实现变化

```cpp
// Robot/instances/dummy_robot.cpp
uint32_t DummyRobot::CommandHandler::Push(const char* _cmd)
{
    // Copy into a fixed-size message so osMessageQueuePut (msg_size=CMD_MAX_LENGTH)
    // never reads past the caller's buffer. No heap allocation involved.
    char msg[CMD_MAX_LENGTH];
    strncpy(msg, _cmd, CMD_MAX_LENGTH - 1);
    msg[CMD_MAX_LENGTH - 1] = 0;

    osStatus_t status = osMessageQueuePut(commandFifo, msg, 0U, 0U);
    if (status == osOK)
        return osMessageQueueGetSpace(commandFifo);

    return 0xFF; // failed
}

bool DummyRobot::CommandHandler::Pop(char* _buffer, uint32_t timeout)
{
    return osMessageQueueGet(commandFifo, _buffer, nullptr, timeout) == osOK;
}
```

- `Push` 先把命令拷进**栈上**完整初始化的 64 字节数组再入队，
  顺手修掉了 4.3 节的越界读：拷贝源是定长、内容确定的本地数组；
- `ParseCommand` 内 4 处 `sscanf(_cmd.c_str(), ...)` 改为 `sscanf(_cmd, ...)`。

```cpp
// UserApp/main.cpp —— 执行线程
void ThreadControlLoopUpdate(void* argument)
{
    char cmd[DummyRobot::CommandHandler::CMD_MAX_LENGTH];   // 栈上缓冲区，循环复用
    for (;;)
    {
        if (dummy.commandHandler.Pop(cmd, osWaitForever))
            dummy.commandHandler.ParseCommand(cmd);
    }
}
```

### 5.4 为什么这样改就好了

| 对比项 | 改造前 | 改造后 |
|---|---|---|
| 每条命令的堆操作 | 2 对 malloc/free，跨两个线程 | **0 次** |
| 是否依赖 newlib malloc 线程安全 | 是（而它没锁） | 否，堆根本不参与 |
| 命令存储位置 | 堆（std::string）+ 队列 | 只用 FreeRTOS 队列（内部自带锁） |
| 越界读隐患 | 有（固定拷 64 字节） | 无（拷贝源是完整初始化的 64 字节数组） |

线程间唯一共享的资源变成了 `commandFifo` 消息队列本身，而 FreeRTOS 队列
内部自带临界区保护——**竞争从根上消失**，50Hz、100Hz 都可以稳定工作。

另外注意：`ascii_protocol.cpp` 中 `Push(_cmd)` 的调用点**一行都不用改**——
`_cmd` 本来就是 `const char*`，以前靠隐式转换成 `std::string`（触发 malloc），
现在直接匹配新接口。这也说明旧写法里那次堆分配有多"隐蔽"。

### 5.5 改造后的数据流总览

```text
USB 收包线程                                命令执行线程
───────────                                ───────────
rx_buf (USB 静态缓冲)
  → parse_buffer (static, 256B)
    → cmd[] (栈上, 257B)
      → msg[] (栈上, 64B, strncpy 定长拷贝)
        → commandFifo (RTOS 队列, 内部拷贝+加锁) ────► cmd[] (执行线程栈上, 64B)
                                                          → sscanf 原地解析
                                                          → MoveJ() / CAN 下发
全程：malloc 次数 = 0
```

---

## 6. 经验总结（给后续开发的建议）

1. **高频路径零动态分配**：命令收发、中断回调、控制环这类路径上，
   一律使用定长数组/环形缓冲，禁用 `std::string`、`new`、STL 容器。
2. **隐式转换要警惕**：`void f(const std::string&)` 接收 `char*` 时会
   悄悄构造临时对象。接口签名用 `const char*` 可以从源头杜绝。
3. **newlib + RTOS 必须补两件事**（如果确实需要多线程用 C 库）：
   - 实现 `__retarget_lock_acquire/__retarget_lock_release`
     （内部用 FreeRTOS mutex），让 malloc/free 线程安全；
   - 在 `FreeRTOSConfig.h` 定义 `configUSE_NEWLIB_REENTRANT 1`，
     让每个任务有独立的 `_reent`。
4. **偶发路径可以放宽**：像 `!STOP` 这类手动低频命令分支里残留的
   `std::string s(_cmd)`，频率极低且在单线程内完成，风险可接受；
   追求极致也可换成 `strstr()`。
5. **崩溃信息会"说话"**：`terminate called after throwing...` 出现在
   串口/USB 输出里，说明固件抛了未捕获异常。顺着 `_write` 的重定向
   就能确认信息来自固件，而不是怀疑上位机。

---

## 附：涉及的源码位置速查

| 内容 | 位置 |
|---|---|
| 命令模式分支（`>`/`&`/`@` 解析与应答） | `Robot/instances/dummy_robot.cpp` `ParseCommand()` |
| CommandHandler 类与 FIFO 定义 | `Robot/instances/dummy_robot.h` |
| USB 收包线程 | `Bsp/communication/interface_usb.cpp` `UsbServerTask()` |
| 命令入队调用点 | `UserApp/protocols/ascii_protocol.cpp` `OnUsbAsciiCmd()` |
| 命令执行线程 | `UserApp/main.cpp` `ThreadControlLoopUpdate()` |
| stderr/printf 重定向到 USB+UART4 | `Bsp/communication/communication.cpp` `_write()` |
| newlib 堆增长 | `Core/Src/syscalls.c` `_sbrk()` |
| 堆大小定义（0x3C00 = 15KB） | `STM32F405RGTx_FLASH.ld` `_Min_Heap_Size` |
