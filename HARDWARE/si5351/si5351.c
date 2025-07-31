// 本文件为Si5351时钟发生器驱动的实现文件，包含主要寄存器定义、初始化、配置、I2C通信等函数实现
// 适用于STM32等MCU通过I2C控制Si5351芯片
// 主要函数均已添加中文注释，便于理解和移植

// vim: set ai et ts=4 sw=4:

// 根据您的MCU更改此处的头文件包含
#include "stm32f4xx_hal.h"


#include <si5351.h>
#define SI5351_ADDRESS 0x60  // SI5351芯片的I2C地址
#define I2C_HANDLE hi2c1     // I2C句柄定义，根据实际使用的I2C外设修改
extern I2C_HandleTypeDef I2C_HANDLE;

// 私有函数声明
void si5351_writeBulk(uint8_t baseaddr, int32_t P1, int32_t P2, int32_t P3, uint8_t divBy4, si5351RDiv_t rdiv);
void si5351_write(uint8_t reg, uint8_t value);

// 参考文档：http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf
enum {
    SI5351_REGISTER_0_DEVICE_STATUS                       = 0,
    SI5351_REGISTER_1_INTERRUPT_STATUS_STICKY             = 1,
    SI5351_REGISTER_2_INTERRUPT_STATUS_MASK               = 2,
    SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL               = 3,
    SI5351_REGISTER_9_OEB_PIN_ENABLE_CONTROL              = 9,
    SI5351_REGISTER_15_PLL_INPUT_SOURCE                   = 15,
    SI5351_REGISTER_16_CLK0_CONTROL                       = 16,
    SI5351_REGISTER_17_CLK1_CONTROL                       = 17,
    SI5351_REGISTER_18_CLK2_CONTROL                       = 18,
    SI5351_REGISTER_19_CLK3_CONTROL                       = 19,
    SI5351_REGISTER_20_CLK4_CONTROL                       = 20,
    SI5351_REGISTER_21_CLK5_CONTROL                       = 21,
    SI5351_REGISTER_22_CLK6_CONTROL                       = 22,
    SI5351_REGISTER_23_CLK7_CONTROL                       = 23,
    SI5351_REGISTER_24_CLK3_0_DISABLE_STATE               = 24,
    SI5351_REGISTER_25_CLK7_4_DISABLE_STATE               = 25,
    SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1           = 42,
    SI5351_REGISTER_43_MULTISYNTH0_PARAMETERS_2           = 43,
    SI5351_REGISTER_44_MULTISYNTH0_PARAMETERS_3           = 44,
    SI5351_REGISTER_45_MULTISYNTH0_PARAMETERS_4           = 45,
    SI5351_REGISTER_46_MULTISYNTH0_PARAMETERS_5           = 46,
    SI5351_REGISTER_47_MULTISYNTH0_PARAMETERS_6           = 47,
    SI5351_REGISTER_48_MULTISYNTH0_PARAMETERS_7           = 48,
    SI5351_REGISTER_49_MULTISYNTH0_PARAMETERS_8           = 49,
    SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1           = 50,
    SI5351_REGISTER_51_MULTISYNTH1_PARAMETERS_2           = 51,
    SI5351_REGISTER_52_MULTISYNTH1_PARAMETERS_3           = 52,
    SI5351_REGISTER_53_MULTISYNTH1_PARAMETERS_4           = 53,
    SI5351_REGISTER_54_MULTISYNTH1_PARAMETERS_5           = 54,
    SI5351_REGISTER_55_MULTISYNTH1_PARAMETERS_6           = 55,
    SI5351_REGISTER_56_MULTISYNTH1_PARAMETERS_7           = 56,
    SI5351_REGISTER_57_MULTISYNTH1_PARAMETERS_8           = 57,
    SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1           = 58,
    SI5351_REGISTER_59_MULTISYNTH2_PARAMETERS_2           = 59,
    SI5351_REGISTER_60_MULTISYNTH2_PARAMETERS_3           = 60,
    SI5351_REGISTER_61_MULTISYNTH2_PARAMETERS_4           = 61,
    SI5351_REGISTER_62_MULTISYNTH2_PARAMETERS_5           = 62,
    SI5351_REGISTER_63_MULTISYNTH2_PARAMETERS_6           = 63,
    SI5351_REGISTER_64_MULTISYNTH2_PARAMETERS_7           = 64,
    SI5351_REGISTER_65_MULTISYNTH2_PARAMETERS_8           = 65,
    SI5351_REGISTER_66_MULTISYNTH3_PARAMETERS_1           = 66,
    SI5351_REGISTER_67_MULTISYNTH3_PARAMETERS_2           = 67,
    SI5351_REGISTER_68_MULTISYNTH3_PARAMETERS_3           = 68,
    SI5351_REGISTER_69_MULTISYNTH3_PARAMETERS_4           = 69,
    SI5351_REGISTER_70_MULTISYNTH3_PARAMETERS_5           = 70,
    SI5351_REGISTER_71_MULTISYNTH3_PARAMETERS_6           = 71,
    SI5351_REGISTER_72_MULTISYNTH3_PARAMETERS_7           = 72,
    SI5351_REGISTER_73_MULTISYNTH3_PARAMETERS_8           = 73,
    SI5351_REGISTER_74_MULTISYNTH4_PARAMETERS_1           = 74,
    SI5351_REGISTER_75_MULTISYNTH4_PARAMETERS_2           = 75,
    SI5351_REGISTER_76_MULTISYNTH4_PARAMETERS_3           = 76,
    SI5351_REGISTER_77_MULTISYNTH4_PARAMETERS_4           = 77,
    SI5351_REGISTER_78_MULTISYNTH4_PARAMETERS_5           = 78,
    SI5351_REGISTER_79_MULTISYNTH4_PARAMETERS_6           = 79,
    SI5351_REGISTER_80_MULTISYNTH4_PARAMETERS_7           = 80,
    SI5351_REGISTER_81_MULTISYNTH4_PARAMETERS_8           = 81,
    SI5351_REGISTER_82_MULTISYNTH5_PARAMETERS_1           = 82,
    SI5351_REGISTER_83_MULTISYNTH5_PARAMETERS_2           = 83,
    SI5351_REGISTER_84_MULTISYNTH5_PARAMETERS_3           = 84,
    SI5351_REGISTER_85_MULTISYNTH5_PARAMETERS_4           = 85,
    SI5351_REGISTER_86_MULTISYNTH5_PARAMETERS_5           = 86,
    SI5351_REGISTER_87_MULTISYNTH5_PARAMETERS_6           = 87,
    SI5351_REGISTER_88_MULTISYNTH5_PARAMETERS_7           = 88,
    SI5351_REGISTER_89_MULTISYNTH5_PARAMETERS_8           = 89,
    SI5351_REGISTER_90_MULTISYNTH6_PARAMETERS             = 90,
    SI5351_REGISTER_91_MULTISYNTH7_PARAMETERS             = 91,
    SI5351_REGISTER_92_CLOCK_6_7_OUTPUT_DIVIDER           = 92,
    SI5351_REGISTER_165_CLK0_INITIAL_PHASE_OFFSET         = 165,
    SI5351_REGISTER_166_CLK1_INITIAL_PHASE_OFFSET         = 166,
    SI5351_REGISTER_167_CLK2_INITIAL_PHASE_OFFSET         = 167,
    SI5351_REGISTER_168_CLK3_INITIAL_PHASE_OFFSET         = 168,
    SI5351_REGISTER_169_CLK4_INITIAL_PHASE_OFFSET         = 169,
    SI5351_REGISTER_170_CLK5_INITIAL_PHASE_OFFSET         = 170,
    SI5351_REGISTER_177_PLL_RESET                         = 177,
    SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE = 183
};

// 晶振负载电容枚举类型，用于设置芯片内部晶振负载电容值
typedef enum {
    SI5351_CRYSTAL_LOAD_6PF  = (1<<6),  // 6pF负载电容
    SI5351_CRYSTAL_LOAD_8PF  = (2<<6),  // 8pF负载电容
    SI5351_CRYSTAL_LOAD_10PF = (3<<6)   // 10pF负载电容
} si5351CrystalLoad_t;

int32_t si5351Correction; // 全局频率校正值

/*
 * @brief 初始化Si5351芯片，必须在使用其他功能前调用。
 * @param correction 频率校正值，表示实际频率与目标频率在100MHz时的差值。
 *        可在较低频率下测量并线性缩放。例如，实际测得10_000_097Hz而目标为10_000_000Hz，则correction=97*10=970。
 *
 * 此函数会：
 * 1. 禁用所有输出（所有CLKx_DIS位置1）
 * 2. 关闭所有输出驱动（所有输出寄存器最高位1）
 * 3. 设置晶振负载电容为10pF
 */
void si5351_Init(int32_t correction) {
    si5351Correction = correction; // 保存校正值

    // 禁用所有输出，将所有CLKx_DIS位置1
    si5351_write(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF);

    // 关闭所有输出驱动（寄存器高位1）
    si5351_write(SI5351_REGISTER_16_CLK0_CONTROL, 0x80);
    si5351_write(SI5351_REGISTER_17_CLK1_CONTROL, 0x80);
    si5351_write(SI5351_REGISTER_18_CLK2_CONTROL, 0x80);
    si5351_write(SI5351_REGISTER_19_CLK3_CONTROL, 0x80);
    si5351_write(SI5351_REGISTER_20_CLK4_CONTROL, 0x80);
    si5351_write(SI5351_REGISTER_21_CLK5_CONTROL, 0x80);
    si5351_write(SI5351_REGISTER_22_CLK6_CONTROL, 0x80);
    si5351_write(SI5351_REGISTER_23_CLK7_CONTROL, 0x80);

    // 设置晶振负载电容为10pF（可根据实际晶振参数调整）
    si5351CrystalLoad_t crystalLoad = SI5351_CRYSTAL_LOAD_10PF;
    si5351_write(SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE, crystalLoad);
}

/*
 * @brief 设置指定PLL的倍频器参数
 * @param pll 要配置的PLL（PLLA或PLLB）
 * @param conf PLL配置参数结构体指针，包含倍频系数的整数和分数部分
 * 
 * 此函数根据配置参数计算PLL寄存器值P1、P2、P3，并写入对应寄存器
 * 计算公式：P1 = 128 * mult + (128 * num)/denom - 512
 *           P2 = (128 * num) % denom
 *           P3 = denom
 */
void si5351_SetupPLL(si5351PLL_t pll, si5351PLLConfig_t* conf) {
    int32_t P1, P2, P3;
    int32_t mult = conf->mult;   // 倍频系数整数部分
    int32_t num = conf->num;     // 倍频系数分数部分分子
    int32_t denom = conf->denom; // 倍频系数分数部分分母

    // 根据SI5351数据手册公式计算PLL参数
    P1 = 128 * mult + (128 * num)/denom - 512;
    // P2 = 128 * num - denom * ((128 * num)/denom);
    P2 = (128 * num) % denom;
    P3 = denom;

    // 获取PLL寄存器的基地址（PLLA基地址26，PLLB基地址34）
    uint8_t baseaddr = (pll == SI5351_PLL_A ? 26 : 34);
    si5351_writeBulk(baseaddr, P1, P2, P3, 0, 0);

    // 复位两个PLL
    si5351_write(SI5351_REGISTER_177_PLL_RESET, (1<<7) | (1<<5) );
}

/*
 * @brief 配置输出通道的PLL源、驱动强度、多同步器分频器、R分频器和相位偏移
 * @param output 输出通道号（0-2）
 * @param pllSource PLL源选择（PLLA或PLLB）
 * @param driveStrength 驱动强度设置
 * @param conf 输出配置参数结构体指针
 * @param phaseOffset 相位偏移值
 * @return 成功返回0，失败返回非0值
 */
int si5351_SetupOutput(uint8_t output, si5351PLL_t pllSource, si5351DriveStrength_t driveStrength, si5351OutputConfig_t* conf, uint8_t phaseOffset) {
    int32_t div = conf->div;
    int32_t num = conf->num;
    int32_t denom = conf->denom;
    uint8_t divBy4 = 0;
    int32_t P1, P2, P3;

    if(output > 2) {
        return 1; // 输出通道号超出范围
    }

    if((!conf->allowIntegerMode) && ((div < 8) || ((div == 8) && (num == 0)))) {
        // 分频值4、6、8只能在整数模式下使用
        return 2;
    }

    if(div == 4) {
        // 特殊的4分频情况，参见AN619 4.1.3节
        P1 = 0;
        P2 = 0;
        P3 = 1;
        divBy4 = 0x3;
    } else {
        P1 = 128 * div + ((128 * num)/denom) - 512;
        // P2 = 128 * num - denom * (128 * num)/denom;
        P2 = (128 * num) % denom;
        P3 = denom;
    }

    // 获取指定通道的寄存器地址
    uint8_t baseaddr = 0;
    uint8_t phaseOffsetRegister = 0;
    uint8_t clkControlRegister = 0;
    switch (output) {
    case 0:
        baseaddr = SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1;
        phaseOffsetRegister = SI5351_REGISTER_165_CLK0_INITIAL_PHASE_OFFSET;
        clkControlRegister = SI5351_REGISTER_16_CLK0_CONTROL;
        break;
    case 1:
        baseaddr = SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1;
        phaseOffsetRegister = SI5351_REGISTER_166_CLK1_INITIAL_PHASE_OFFSET;
        clkControlRegister = SI5351_REGISTER_17_CLK1_CONTROL;
        break;
    case 2:
        baseaddr = SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1;
        phaseOffsetRegister = SI5351_REGISTER_167_CLK2_INITIAL_PHASE_OFFSET;
        clkControlRegister = SI5351_REGISTER_18_CLK2_CONTROL;
        break;
    }

    uint8_t clkControl = 0x0C | driveStrength; // 时钟不反相，上电状态
    if(pllSource == SI5351_PLL_B) {
        clkControl |= (1 << 5); // 使用PLLB
    }

    if((conf->allowIntegerMode) && ((num == 0)||(div == 4))) {
        // 使用整数模式
        clkControl |= (1 << 6);
    }

    si5351_write(clkControlRegister, clkControl);
    si5351_writeBulk(baseaddr, P1, P2, P3, divBy4, conf->rdiv);
    si5351_write(phaseOffsetRegister, (phaseOffset & 0x7F));

    return 0;
}

/*
 * @brief 计算指定频率下的PLL、多同步器和R分频器设置
 * @param Fclk 目标频率，范围[8_000, 160_000_000] Hz
 * @param pll_conf 输出PLL配置参数
 * @param out_conf 输出分频器配置参数
 * 
 * 假设校正值正确，实际频率与目标频率的差值将小于6Hz
 */
void si5351_Calc(int32_t Fclk, si5351PLLConfig_t* pll_conf, si5351OutputConfig_t* out_conf) {
    if(Fclk < 8000) Fclk = 8000;
    else if(Fclk > 160000000) Fclk = 160000000;

    out_conf->allowIntegerMode = 1;

    if(Fclk < 1000000) {
        // 对于[8_000, 500_000]范围内的频率，可以使用si5351_Calc(Fclk*64, ...)和SI5351_R_DIV_64
        // 实际上，对于任何低于1MHz的频率都值得这样做，因为这会减少误差
        Fclk *= 64;
        out_conf->rdiv = SI5351_R_DIV_64;
    } else {
        out_conf->rdiv = SI5351_R_DIV_1;
    }

    // 在确定rdiv之后应用校正
    Fclk = Fclk - (int32_t)((((double)Fclk)/100000000.0)*((double)si5351Correction));

    // 这里我们寻找整数值a,b,c,x,y,z，使得：
    // N = a + b / c    # PLL设置
    // M = x + y / z    # 多同步器设置
    // Fclk = Fxtal * N / M
    // N in [24, 36]
    // M in [8, 1800] or M in {4,6}
    // b < c, y < z
    // b,c,y,z <= 2**20
    // c, z != 0
    // 对于[500K, 160MHz]范围内的任何Fclk，此算法都能找到一个解决方案
    // 使得abs(Ffound - Fclk) <= 6 Hz

    const int32_t Fxtal = 25000000;
    int32_t a, b, c, x, y, z, t;

    if(Fclk < 81000000) {
        // 对于0.5..112.5 MHz范围有效
        // 但是在81 MHz以上误差会大于6 Hz
        a = 36; // PLL运行在900 MHz
        b = 0;
        c = 1;
        int32_t Fpll = 900000000;
        x = Fpll/Fclk;
        t = (Fclk >> 20) + 1;
        y = (Fpll % Fclk) / t;
        z = Fclk / t;
    } else {
        // 对于75..160 MHz范围有效
        if(Fclk >= 150000000) {
            x = 4;
        } else if (Fclk >= 100000000) {
            x = 6;
        } else {
            x = 8;
        }
        y = 0;
        z = 1;
        
        int32_t numerator = x*Fclk;
        a = numerator/Fxtal;
        t = (Fxtal >> 20) + 1;
        b = (numerator % Fxtal) / t;
        c = Fxtal / t;
    }

    pll_conf->mult = a;
    pll_conf->num = b;
    pll_conf->denom = c;
    out_conf->div = x;
    out_conf->num = y;
    out_conf->denom = z;
}

/*
 * @brief si5351_CalcIQ用于计算两个通道间90°相位差的PLL和多同步器参数
 * @param Fclk 目标频率，范围[1.4MHz, 100MHz]
 * @param pll_conf 输出PLL配置参数
 * @param out_conf 输出多同步器配置参数
 * 
 * 要实现90°相位差，两个通道的phaseOffset分别设为0和(uint8_t)out_conf.div
 * 且必须使用同一个PLL。频率范围1.4MHz到100MHz。
 * 假设校正值正确，实际频率与目标频率的差值将小于4Hz
 */
void si5351_CalcIQ(int32_t Fclk, si5351PLLConfig_t* pll_conf, si5351OutputConfig_t* out_conf) {
    const int32_t Fxtal = 25000000;
    int32_t Fpll;

    if(Fclk < 1400000) Fclk = 1400000;
    else if(Fclk > 100000000) Fclk = 100000000;

    // 应用校正
    Fclk = Fclk - ((Fclk/1000000)*si5351Correction)/100;

    // 禁用整数模式
    out_conf->allowIntegerMode = 0;

    // 使用R分频器会改变相位偏移，而AN619文档没有对此变化给出任何保证
    out_conf->rdiv = 0;

    if(Fclk < 4900000) {
        // 小技巧：让PLL运行在600MHz以下以覆盖1.4MHz到4.725MHz的范围
        // AN619并没有明确说PLL不能运行在600MHz以下
        // 实验表明当PLL运行在177MHz以下时会变得不稳定
        // 这将Fclk限制在177/127=1.4MHz
        out_conf->div = 127;
    } else if(Fclk < 8000000) {
        out_conf->div = 625000000 / Fclk;
    } else {
        out_conf->div = 900000000 / Fclk;
    }
    out_conf->num = 0;
    out_conf->denom = 1;

    Fpll = Fclk * out_conf->div;
    pll_conf->mult = Fpll / Fxtal;
    pll_conf->num = (Fpll % Fxtal) / 24;
    pll_conf->denom = Fxtal / 24; // 分母不能超过0xFFFFF
}

/*
 * @brief 为CLK0设置指定的频率和驱动强度，使用PLLA
 * @param Fclk 目标频率
 * @param driveStrength 驱动强度设置
 */
void si5351_SetupCLK0(int32_t Fclk, si5351DriveStrength_t driveStrength) {
	si5351PLLConfig_t pll_conf;
	si5351OutputConfig_t out_conf;

	si5351_Calc(Fclk, &pll_conf, &out_conf);
	si5351_SetupPLL(SI5351_PLL_A, &pll_conf);
	si5351_SetupOutput(0, SI5351_PLL_A, driveStrength, &out_conf, 0);
}

/*
 * @brief 为CLK2设置指定的频率和驱动强度，使用PLLB
 * @param Fclk 目标频率
 * @param driveStrength 驱动强度设置
 */
void si5351_SetupCLK2(int32_t Fclk, si5351DriveStrength_t driveStrength) {
	si5351PLLConfig_t pll_conf;
	si5351OutputConfig_t out_conf;

	si5351_Calc(Fclk, &pll_conf, &out_conf);
	si5351_SetupPLL(SI5351_PLL_B, &pll_conf);
	si5351_SetupOutput(2, SI5351_PLL_B, driveStrength, &out_conf, 0);
}

/*
 * @brief 根据提供的位掩码使能或禁用输出
 * 例如：
 * si5351_EnableOutputs(1 << 0) 使能CLK0，禁用CLK1和CLK2
 * si5351_EnableOutputs((1 << 2) | (1 << 0)) 使能CLK0和CLK2，禁用CLK1
 * @param enabled 输出使能位掩码
 */
void si5351_EnableOutputs(uint8_t enabled) {
    si5351_write(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, ~enabled);
}

/*
 * @brief 通过I2C向寄存器写入8位值
 * @param reg 寄存器地址
 * @param value 要写入的值
 */
void si5351_write(uint8_t reg, uint8_t value) {
    while (HAL_I2C_IsDeviceReady(&I2C_HANDLE, (uint16_t)(SI5351_ADDRESS<<1), 3, HAL_MAX_DELAY) != HAL_OK) { }

    HAL_I2C_Mem_Write(&I2C_HANDLE,                  // I2C句柄
                      (uint8_t)(SI5351_ADDRESS<<1), // I2C地址，左对齐
                      (uint8_t)reg,                 // 寄存器地址
                      I2C_MEMADD_SIZE_8BIT,         // SI5351使用8位寄存器地址
                      (uint8_t*)(&value),           // 要写入的数据
                      1,                            // 写入字节数
                      HAL_MAX_DELAY);               // 超时时间
}

/*
 * @brief _SetupPLL和_SetupOutput的公共代码，用于批量写入寄存器
 * @param baseaddr 基地址
 * @param P1 PLL参数P1
 * @param P2 PLL参数P2
 * @param P3 PLL参数P3
 * @param divBy4 4分频标志
 * @param rdiv R分频器设置
 */
void si5351_writeBulk(uint8_t baseaddr, int32_t P1, int32_t P2, int32_t P3, uint8_t divBy4, si5351RDiv_t rdiv) {
    si5351_write(baseaddr,   (P3 >> 8) & 0xFF);
    si5351_write(baseaddr+1, P3 & 0xFF);
    si5351_write(baseaddr+2, ((P1 >> 16) & 0x3) | ((divBy4 & 0x3) << 2) | ((rdiv & 0x7) << 4));
    si5351_write(baseaddr+3, (P1 >> 8) & 0xFF);
    si5351_write(baseaddr+4, P1 & 0xFF);
    si5351_write(baseaddr+5, ((P3 >> 12) & 0xF0) | ((P2 >> 16) & 0xF));
    si5351_write(baseaddr+6, (P2 >> 8) & 0xFF);
    si5351_write(baseaddr+7, P2 & 0xFF);
}
