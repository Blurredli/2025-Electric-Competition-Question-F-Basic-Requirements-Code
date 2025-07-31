// 本文件为Si5351时钟发生器驱动的头文件，包含主要类型定义和接口声明
// 适用于STM32等MCU通过I2C控制Si5351芯片
// 结构体和函数均已添加中文注释，便于理解和移植

// vim: set ai et ts=4 sw=4:
#ifndef _SI5351_H_
#define _SI5351_H_



// SI5351 PLL选择枚举类型，表示可选的两个锁相环（PLLA/PLLB），用于频率合成
// SI5351_PLL_A：第一个PLL，SI5351_PLL_B：第二个PLL
typedef enum {
    SI5351_PLL_A = 0,  // PLL A
    SI5351_PLL_B,      // PLL B
} si5351PLL_t;

// 输出分频器R分频比枚举类型，设置输出端口的R分频器分频比
// 例如SI5351_R_DIV_8表示R分频器分频比为8
typedef enum {
    SI5351_R_DIV_1   = 0,  // R分频比为1
    SI5351_R_DIV_2   = 1,  // R分频比为2
    SI5351_R_DIV_4   = 2,  // R分频比为4
    SI5351_R_DIV_8   = 3,  // R分频比为8
    SI5351_R_DIV_16  = 4,  // R分频比为16
    SI5351_R_DIV_32  = 5,  // R分频比为32
    SI5351_R_DIV_64  = 6,  // R分频比为64
    SI5351_R_DIV_128 = 7,  // R分频比为128
} si5351RDiv_t;

// 输出驱动强度枚举，表示不同的输出电流驱动能力，对应不同的输出功率
// 例如SI5351_DRIVE_STRENGTH_8MA表示最大8mA驱动能力
typedef enum {
    SI5351_DRIVE_STRENGTH_2MA = 0x00, //  ~ 2.2 dBm输出功率
    SI5351_DRIVE_STRENGTH_4MA = 0x01, //  ~ 7.5 dBm输出功率
    SI5351_DRIVE_STRENGTH_6MA = 0x02, //  ~ 9.5 dBm输出功率
    SI5351_DRIVE_STRENGTH_8MA = 0x03, // ~ 10.7 dBm输出功率
} si5351DriveStrength_t;

// PLL配置结构体，包含PLL倍频系数的整数和分数部分
// mult：倍频系数整数部分，num/denom：分数部分
typedef struct {
    int32_t mult;  // 倍频系数的整数部分
    int32_t num;   // 倍频系数分数部分的分子
    int32_t denom; // 倍频系数分数部分的分母
} si5351PLLConfig_t;

// 输出配置结构体，包含分频器参数和R分频器设置
// allowIntegerMode：是否允许整数分频模式
// div/num/denom：分频系数，rdiv：R分频器设置
typedef struct {
    uint8_t allowIntegerMode; // 是否允许整数模式
    int32_t div;              // 分频系数的整数部分
    int32_t num;              // 分频系数分数部分的分子
    int32_t denom;            // 分频系数分数部分的分母
    si5351RDiv_t rdiv;        // R分频器设置
} si5351OutputConfig_t;

/*
 * 基本接口只允许使用CLK0和CLK2。
 * 此接口为CLK0和CLK2使用独立的PLL，因此可以独立地改变频率。
 * 如果还需要CLK1，一个PLL必须在两个CLKx之间共享，这会使配置变得更复杂。
 * 选择CLK0和CLK2是因为在常见的Si5351模块上它们的物理位置相距较远，
 * 这使得使用它们比使用CLK0和CLK1更方便。
 *
 * si5351_Init：初始化SI5351芯片，correction为频率校正值
 * si5351_SetupCLK0/CLK2：设置CLK0/CLK2输出频率和驱动能力
 * si5351_EnableOutputs：使能或关闭输出端口
 */
void si5351_Init(int32_t correction);
void si5351_SetupCLK0(int32_t Fclk, si5351DriveStrength_t driveStrength);
void si5351_SetupCLK2(int32_t Fclk, si5351DriveStrength_t driveStrength);
void si5351_EnableOutputs(uint8_t enabled);

/*
 * 高级接口说明：
 * 1. 如果需要同时使用CLK0、CLK1和CLK2，或需要两个通道间90°相位差，使用此接口
 * 2. si5351_Calc始终为81MHz以下频率使用900MHz的PLL，该PLL可被所有<=81MHz的输出共享
 * 3. 也可修改源码以支持<=112.5MHz共用一个PLL，但最大误差会增大到13Hz
 *
 * si5351_Calc：计算PLL和输出分频参数
 */
void si5351_Calc(int32_t Fclk, si5351PLLConfig_t* pll_conf, si5351OutputConfig_t* out_conf);

/*
 * si5351_CalcIQ：用于计算两个通道间90°相位差的PLL和分频参数
 * 只要两个通道phaseOffset分别为0和out_conf.div，且共用同一个PLL即可实现90°移相
 */
void si5351_CalcIQ(int32_t Fclk, si5351PLLConfig_t* pll_conf, si5351OutputConfig_t* out_conf);

void si5351_SetupPLL(si5351PLL_t pll, si5351PLLConfig_t* conf);
int si5351_SetupOutput(uint8_t output, si5351PLL_t pllSource, si5351DriveStrength_t driveStength, si5351OutputConfig_t* conf, uint8_t phaseOffset);

#endif
