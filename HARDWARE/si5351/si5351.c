// ���ļ�ΪSi5351ʱ�ӷ�����������ʵ���ļ���������Ҫ�Ĵ������塢��ʼ�������á�I2Cͨ�ŵȺ���ʵ��
// ������STM32��MCUͨ��I2C����Si5351оƬ
// ��Ҫ���������������ע�ͣ�����������ֲ

// vim: set ai et ts=4 sw=4:

// ��������MCU���Ĵ˴���ͷ�ļ�����
#include "stm32f4xx_hal.h"


#include <si5351.h>
#define SI5351_ADDRESS 0x60  // SI5351оƬ��I2C��ַ
#define I2C_HANDLE hi2c1     // I2C������壬����ʵ��ʹ�õ�I2C�����޸�
extern I2C_HandleTypeDef I2C_HANDLE;

// ˽�к�������
void si5351_writeBulk(uint8_t baseaddr, int32_t P1, int32_t P2, int32_t P3, uint8_t divBy4, si5351RDiv_t rdiv);
void si5351_write(uint8_t reg, uint8_t value);

// �ο��ĵ���http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf
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

// �����ص���ö�����ͣ���������оƬ�ڲ������ص���ֵ
typedef enum {
    SI5351_CRYSTAL_LOAD_6PF  = (1<<6),  // 6pF���ص���
    SI5351_CRYSTAL_LOAD_8PF  = (2<<6),  // 8pF���ص���
    SI5351_CRYSTAL_LOAD_10PF = (3<<6)   // 10pF���ص���
} si5351CrystalLoad_t;

int32_t si5351Correction; // ȫ��Ƶ��У��ֵ

/*
 * @brief ��ʼ��Si5351оƬ��������ʹ����������ǰ���á�
 * @param correction Ƶ��У��ֵ����ʾʵ��Ƶ����Ŀ��Ƶ����100MHzʱ�Ĳ�ֵ��
 *        ���ڽϵ�Ƶ���²������������š����磬ʵ�ʲ��10_000_097Hz��Ŀ��Ϊ10_000_000Hz����correction=97*10=970��
 *
 * �˺����᣺
 * 1. �����������������CLKx_DISλ��1��
 * 2. �ر����������������������Ĵ������λ1��
 * 3. ���þ����ص���Ϊ10pF
 */
void si5351_Init(int32_t correction) {
    si5351Correction = correction; // ����У��ֵ

    // �������������������CLKx_DISλ��1
    si5351_write(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF);

    // �ر���������������Ĵ�����λ1��
    si5351_write(SI5351_REGISTER_16_CLK0_CONTROL, 0x80);
    si5351_write(SI5351_REGISTER_17_CLK1_CONTROL, 0x80);
    si5351_write(SI5351_REGISTER_18_CLK2_CONTROL, 0x80);
    si5351_write(SI5351_REGISTER_19_CLK3_CONTROL, 0x80);
    si5351_write(SI5351_REGISTER_20_CLK4_CONTROL, 0x80);
    si5351_write(SI5351_REGISTER_21_CLK5_CONTROL, 0x80);
    si5351_write(SI5351_REGISTER_22_CLK6_CONTROL, 0x80);
    si5351_write(SI5351_REGISTER_23_CLK7_CONTROL, 0x80);

    // ���þ����ص���Ϊ10pF���ɸ���ʵ�ʾ������������
    si5351CrystalLoad_t crystalLoad = SI5351_CRYSTAL_LOAD_10PF;
    si5351_write(SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE, crystalLoad);
}

/*
 * @brief ����ָ��PLL�ı�Ƶ������
 * @param pll Ҫ���õ�PLL��PLLA��PLLB��
 * @param conf PLL���ò����ṹ��ָ�룬������Ƶϵ���������ͷ�������
 * 
 * �˺����������ò�������PLL�Ĵ���ֵP1��P2��P3����д���Ӧ�Ĵ���
 * ���㹫ʽ��P1 = 128 * mult + (128 * num)/denom - 512
 *           P2 = (128 * num) % denom
 *           P3 = denom
 */
void si5351_SetupPLL(si5351PLL_t pll, si5351PLLConfig_t* conf) {
    int32_t P1, P2, P3;
    int32_t mult = conf->mult;   // ��Ƶϵ����������
    int32_t num = conf->num;     // ��Ƶϵ���������ַ���
    int32_t denom = conf->denom; // ��Ƶϵ���������ַ�ĸ

    // ����SI5351�����ֲṫʽ����PLL����
    P1 = 128 * mult + (128 * num)/denom - 512;
    // P2 = 128 * num - denom * ((128 * num)/denom);
    P2 = (128 * num) % denom;
    P3 = denom;

    // ��ȡPLL�Ĵ����Ļ���ַ��PLLA����ַ26��PLLB����ַ34��
    uint8_t baseaddr = (pll == SI5351_PLL_A ? 26 : 34);
    si5351_writeBulk(baseaddr, P1, P2, P3, 0, 0);

    // ��λ����PLL
    si5351_write(SI5351_REGISTER_177_PLL_RESET, (1<<7) | (1<<5) );
}

/*
 * @brief �������ͨ����PLLԴ������ǿ�ȡ���ͬ������Ƶ����R��Ƶ������λƫ��
 * @param output ���ͨ���ţ�0-2��
 * @param pllSource PLLԴѡ��PLLA��PLLB��
 * @param driveStrength ����ǿ������
 * @param conf ������ò����ṹ��ָ��
 * @param phaseOffset ��λƫ��ֵ
 * @return �ɹ�����0��ʧ�ܷ��ط�0ֵ
 */
int si5351_SetupOutput(uint8_t output, si5351PLL_t pllSource, si5351DriveStrength_t driveStrength, si5351OutputConfig_t* conf, uint8_t phaseOffset) {
    int32_t div = conf->div;
    int32_t num = conf->num;
    int32_t denom = conf->denom;
    uint8_t divBy4 = 0;
    int32_t P1, P2, P3;

    if(output > 2) {
        return 1; // ���ͨ���ų�����Χ
    }

    if((!conf->allowIntegerMode) && ((div < 8) || ((div == 8) && (num == 0)))) {
        // ��Ƶֵ4��6��8ֻ��������ģʽ��ʹ��
        return 2;
    }

    if(div == 4) {
        // �����4��Ƶ������μ�AN619 4.1.3��
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

    // ��ȡָ��ͨ���ļĴ�����ַ
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

    uint8_t clkControl = 0x0C | driveStrength; // ʱ�Ӳ����࣬�ϵ�״̬
    if(pllSource == SI5351_PLL_B) {
        clkControl |= (1 << 5); // ʹ��PLLB
    }

    if((conf->allowIntegerMode) && ((num == 0)||(div == 4))) {
        // ʹ������ģʽ
        clkControl |= (1 << 6);
    }

    si5351_write(clkControlRegister, clkControl);
    si5351_writeBulk(baseaddr, P1, P2, P3, divBy4, conf->rdiv);
    si5351_write(phaseOffsetRegister, (phaseOffset & 0x7F));

    return 0;
}

/*
 * @brief ����ָ��Ƶ���µ�PLL����ͬ������R��Ƶ������
 * @param Fclk Ŀ��Ƶ�ʣ���Χ[8_000, 160_000_000] Hz
 * @param pll_conf ���PLL���ò���
 * @param out_conf �����Ƶ�����ò���
 * 
 * ����У��ֵ��ȷ��ʵ��Ƶ����Ŀ��Ƶ�ʵĲ�ֵ��С��6Hz
 */
void si5351_Calc(int32_t Fclk, si5351PLLConfig_t* pll_conf, si5351OutputConfig_t* out_conf) {
    if(Fclk < 8000) Fclk = 8000;
    else if(Fclk > 160000000) Fclk = 160000000;

    out_conf->allowIntegerMode = 1;

    if(Fclk < 1000000) {
        // ����[8_000, 500_000]��Χ�ڵ�Ƶ�ʣ�����ʹ��si5351_Calc(Fclk*64, ...)��SI5351_R_DIV_64
        // ʵ���ϣ������κε���1MHz��Ƶ�ʶ�ֵ������������Ϊ���������
        Fclk *= 64;
        out_conf->rdiv = SI5351_R_DIV_64;
    } else {
        out_conf->rdiv = SI5351_R_DIV_1;
    }

    // ��ȷ��rdiv֮��Ӧ��У��
    Fclk = Fclk - (int32_t)((((double)Fclk)/100000000.0)*((double)si5351Correction));

    // ��������Ѱ������ֵa,b,c,x,y,z��ʹ�ã�
    // N = a + b / c    # PLL����
    // M = x + y / z    # ��ͬ��������
    // Fclk = Fxtal * N / M
    // N in [24, 36]
    // M in [8, 1800] or M in {4,6}
    // b < c, y < z
    // b,c,y,z <= 2**20
    // c, z != 0
    // ����[500K, 160MHz]��Χ�ڵ��κ�Fclk�����㷨�����ҵ�һ���������
    // ʹ��abs(Ffound - Fclk) <= 6 Hz

    const int32_t Fxtal = 25000000;
    int32_t a, b, c, x, y, z, t;

    if(Fclk < 81000000) {
        // ����0.5..112.5 MHz��Χ��Ч
        // ������81 MHz�����������6 Hz
        a = 36; // PLL������900 MHz
        b = 0;
        c = 1;
        int32_t Fpll = 900000000;
        x = Fpll/Fclk;
        t = (Fclk >> 20) + 1;
        y = (Fpll % Fclk) / t;
        z = Fclk / t;
    } else {
        // ����75..160 MHz��Χ��Ч
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
 * @brief si5351_CalcIQ���ڼ�������ͨ����90����λ���PLL�Ͷ�ͬ��������
 * @param Fclk Ŀ��Ƶ�ʣ���Χ[1.4MHz, 100MHz]
 * @param pll_conf ���PLL���ò���
 * @param out_conf �����ͬ�������ò���
 * 
 * Ҫʵ��90����λ�����ͨ����phaseOffset�ֱ���Ϊ0��(uint8_t)out_conf.div
 * �ұ���ʹ��ͬһ��PLL��Ƶ�ʷ�Χ1.4MHz��100MHz��
 * ����У��ֵ��ȷ��ʵ��Ƶ����Ŀ��Ƶ�ʵĲ�ֵ��С��4Hz
 */
void si5351_CalcIQ(int32_t Fclk, si5351PLLConfig_t* pll_conf, si5351OutputConfig_t* out_conf) {
    const int32_t Fxtal = 25000000;
    int32_t Fpll;

    if(Fclk < 1400000) Fclk = 1400000;
    else if(Fclk > 100000000) Fclk = 100000000;

    // Ӧ��У��
    Fclk = Fclk - ((Fclk/1000000)*si5351Correction)/100;

    // ��������ģʽ
    out_conf->allowIntegerMode = 0;

    // ʹ��R��Ƶ����ı���λƫ�ƣ���AN619�ĵ�û�жԴ˱仯�����κα�֤
    out_conf->rdiv = 0;

    if(Fclk < 4900000) {
        // С���ɣ���PLL������600MHz�����Ը���1.4MHz��4.725MHz�ķ�Χ
        // AN619��û����ȷ˵PLL����������600MHz����
        // ʵ�������PLL������177MHz����ʱ���ò��ȶ�
        // �⽫Fclk������177/127=1.4MHz
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
    pll_conf->denom = Fxtal / 24; // ��ĸ���ܳ���0xFFFFF
}

/*
 * @brief ΪCLK0����ָ����Ƶ�ʺ�����ǿ�ȣ�ʹ��PLLA
 * @param Fclk Ŀ��Ƶ��
 * @param driveStrength ����ǿ������
 */
void si5351_SetupCLK0(int32_t Fclk, si5351DriveStrength_t driveStrength) {
	si5351PLLConfig_t pll_conf;
	si5351OutputConfig_t out_conf;

	si5351_Calc(Fclk, &pll_conf, &out_conf);
	si5351_SetupPLL(SI5351_PLL_A, &pll_conf);
	si5351_SetupOutput(0, SI5351_PLL_A, driveStrength, &out_conf, 0);
}

/*
 * @brief ΪCLK2����ָ����Ƶ�ʺ�����ǿ�ȣ�ʹ��PLLB
 * @param Fclk Ŀ��Ƶ��
 * @param driveStrength ����ǿ������
 */
void si5351_SetupCLK2(int32_t Fclk, si5351DriveStrength_t driveStrength) {
	si5351PLLConfig_t pll_conf;
	si5351OutputConfig_t out_conf;

	si5351_Calc(Fclk, &pll_conf, &out_conf);
	si5351_SetupPLL(SI5351_PLL_B, &pll_conf);
	si5351_SetupOutput(2, SI5351_PLL_B, driveStrength, &out_conf, 0);
}

/*
 * @brief �����ṩ��λ����ʹ�ܻ�������
 * ���磺
 * si5351_EnableOutputs(1 << 0) ʹ��CLK0������CLK1��CLK2
 * si5351_EnableOutputs((1 << 2) | (1 << 0)) ʹ��CLK0��CLK2������CLK1
 * @param enabled ���ʹ��λ����
 */
void si5351_EnableOutputs(uint8_t enabled) {
    si5351_write(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, ~enabled);
}

/*
 * @brief ͨ��I2C��Ĵ���д��8λֵ
 * @param reg �Ĵ�����ַ
 * @param value Ҫд���ֵ
 */
void si5351_write(uint8_t reg, uint8_t value) {
    while (HAL_I2C_IsDeviceReady(&I2C_HANDLE, (uint16_t)(SI5351_ADDRESS<<1), 3, HAL_MAX_DELAY) != HAL_OK) { }

    HAL_I2C_Mem_Write(&I2C_HANDLE,                  // I2C���
                      (uint8_t)(SI5351_ADDRESS<<1), // I2C��ַ�������
                      (uint8_t)reg,                 // �Ĵ�����ַ
                      I2C_MEMADD_SIZE_8BIT,         // SI5351ʹ��8λ�Ĵ�����ַ
                      (uint8_t*)(&value),           // Ҫд�������
                      1,                            // д���ֽ���
                      HAL_MAX_DELAY);               // ��ʱʱ��
}

/*
 * @brief _SetupPLL��_SetupOutput�Ĺ������룬��������д��Ĵ���
 * @param baseaddr ����ַ
 * @param P1 PLL����P1
 * @param P2 PLL����P2
 * @param P3 PLL����P3
 * @param divBy4 4��Ƶ��־
 * @param rdiv R��Ƶ������
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
