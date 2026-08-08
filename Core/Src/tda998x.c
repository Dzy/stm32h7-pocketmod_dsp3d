#include <stdint.h>
#include "i2c.h"
#include "ltdc.h"

extern LTDC_HandleTypeDef hltdc;
extern const LTDCSYNC_t LTDCSYNC[];

#define REG(page, addr) (((page) << 8) | (addr))
#define REG2ADDR(reg)   ((reg) & 0xff)
#define REG2PAGE(reg)   (((reg) >> 8) & 0xff)

#define BIT(x) 1<<x

#define REG_CURPAGE               0xff                

#define I2C_ADDRESS_TDA19988_CEC     (0x34)<<1
#define I2C_ADDRESS_TDA19988_HDMI    (0x70)<<1
#define I2C_ADDRESS_TDA (0x70<<1)

#define REG_VERSION_LSB           REG(0x00, 0x00)     
#define REG_MAIN_CNTRL0           REG(0x00, 0x01)     
# define MAIN_CNTRL0_SR           (1 << 0)
# define MAIN_CNTRL0_DECS         (1 << 1)
# define MAIN_CNTRL0_DEHS         (1 << 2)
# define MAIN_CNTRL0_CECS         (1 << 3)
# define MAIN_CNTRL0_CEHS         (1 << 4)
# define MAIN_CNTRL0_SCALER       (1 << 7)
#define REG_VERSION_MSB           REG(0x00, 0x02)     
#define REG_SOFTRESET             REG(0x00, 0x0a)     
# define SOFTRESET_AUDIO          (1 << 0)
# define SOFTRESET_I2C_MASTER     (1 << 1)
#define REG_DDC_DISABLE           REG(0x00, 0x0b)     
#define REG_CCLK_ON               REG(0x00, 0x0c)     
#define REG_I2C_MASTER            REG(0x00, 0x0d)     
# define I2C_MASTER_DIS_MM        (1 << 0)
# define I2C_MASTER_DIS_FILT      (1 << 1)
# define I2C_MASTER_APP_STRT_LAT  (1 << 2)
#define REG_FEAT_POWERDOWN        REG(0x00, 0x0e)     
# define FEAT_POWERDOWN_PREFILT   BIT(0)
# define FEAT_POWERDOWN_CSC       BIT(1)
# define FEAT_POWERDOWN_SPDIF     (1 << 3)
#define REG_INT_FLAGS_0           REG(0x00, 0x0f)     
#define REG_INT_FLAGS_1           REG(0x00, 0x10)     
#define REG_INT_FLAGS_2           REG(0x00, 0x11)     
# define INT_FLAGS_2_EDID_BLK_RD  (1 << 1)
#define REG_ENA_ACLK              REG(0x00, 0x16)     
#define REG_ENA_VP_0              REG(0x00, 0x18)     
#define REG_ENA_VP_1              REG(0x00, 0x19)     
#define REG_ENA_VP_2              REG(0x00, 0x1a)     
#define REG_ENA_AP                REG(0x00, 0x1e)     
#define REG_VIP_CNTRL_0           REG(0x00, 0x20)     
# define VIP_CNTRL_0_MIRR_A       (1 << 7)
# define VIP_CNTRL_0_SWAP_A(x)    (((x) & 7) << 4)
# define VIP_CNTRL_0_MIRR_B       (1 << 3)
# define VIP_CNTRL_0_SWAP_B(x)    (((x) & 7) << 0)
#define REG_VIP_CNTRL_1           REG(0x00, 0x21)     
# define VIP_CNTRL_1_MIRR_C       (1 << 7)
# define VIP_CNTRL_1_SWAP_C(x)    (((x) & 7) << 4)
# define VIP_CNTRL_1_MIRR_D       (1 << 3)
# define VIP_CNTRL_1_SWAP_D(x)    (((x) & 7) << 0)
#define REG_VIP_CNTRL_2           REG(0x00, 0x22)     
# define VIP_CNTRL_2_MIRR_E       (1 << 7)
# define VIP_CNTRL_2_SWAP_E(x)    (((x) & 7) << 4)
# define VIP_CNTRL_2_MIRR_F       (1 << 3)
# define VIP_CNTRL_2_SWAP_F(x)    (((x) & 7) << 0)
#define REG_VIP_CNTRL_3           REG(0x00, 0x23)     
# define VIP_CNTRL_3_X_TGL        (1 << 0)
# define VIP_CNTRL_3_H_TGL        (1 << 1)
# define VIP_CNTRL_3_V_TGL        (1 << 2)
# define VIP_CNTRL_3_EMB          (1 << 3)
# define VIP_CNTRL_3_SYNC_DE      (1 << 4)
# define VIP_CNTRL_3_SYNC_HS      (1 << 5)
# define VIP_CNTRL_3_DE_INT       (1 << 6)
# define VIP_CNTRL_3_EDGE         (1 << 7)
#define REG_VIP_CNTRL_4           REG(0x00, 0x24)     
# define VIP_CNTRL_4_BLC(x)       (((x) & 3) << 0)
# define VIP_CNTRL_4_BLANKIT(x)   (((x) & 3) << 2)
# define VIP_CNTRL_4_CCIR656      (1 << 4)
# define VIP_CNTRL_4_656_ALT      (1 << 5)
# define VIP_CNTRL_4_TST_656      (1 << 6)
# define VIP_CNTRL_4_TST_PAT      (1 << 7)
#define REG_VIP_CNTRL_5           REG(0x00, 0x25)     
# define VIP_CNTRL_5_CKCASE       (1 << 0)
# define VIP_CNTRL_5_SP_CNT(x)    (((x) & 3) << 1)
#define REG_MUX_AP                REG(0x00, 0x26)     
# define MUX_AP_SELECT_I2S    0x64
# define MUX_AP_SELECT_SPDIF      0x40
#define REG_MUX_VP_VIP_OUT        REG(0x00, 0x27)     
#define REG_MAT_CONTRL            REG(0x00, 0x80)     
# define MAT_CONTRL_MAT_SC(x)     (((x) & 3) << 0)
# define MAT_CONTRL_MAT_BP        (1 << 2)
#define REG_VIDFORMAT             REG(0x00, 0xa0)     
#define REG_REFPIX_MSB            REG(0x00, 0xa1)     
#define REG_REFPIX_LSB            REG(0x00, 0xa2)     
#define REG_REFLINE_MSB           REG(0x00, 0xa3)     
#define REG_REFLINE_LSB           REG(0x00, 0xa4)     
#define REG_NPIX_MSB              REG(0x00, 0xa5)     
#define REG_NPIX_LSB              REG(0x00, 0xa6)     
#define REG_NLINE_MSB             REG(0x00, 0xa7)     
#define REG_NLINE_LSB             REG(0x00, 0xa8)     
#define REG_VS_LINE_STRT_1_MSB    REG(0x00, 0xa9)     
#define REG_VS_LINE_STRT_1_LSB    REG(0x00, 0xaa)     
#define REG_VS_PIX_STRT_1_MSB     REG(0x00, 0xab)     
#define REG_VS_PIX_STRT_1_LSB     REG(0x00, 0xac)     
#define REG_VS_LINE_END_1_MSB     REG(0x00, 0xad)     
#define REG_VS_LINE_END_1_LSB     REG(0x00, 0xae)     
#define REG_VS_PIX_END_1_MSB      REG(0x00, 0xaf)     
#define REG_VS_PIX_END_1_LSB      REG(0x00, 0xb0)     
#define REG_VS_LINE_STRT_2_MSB    REG(0x00, 0xb1)     
#define REG_VS_LINE_STRT_2_LSB    REG(0x00, 0xb2)     
#define REG_VS_PIX_STRT_2_MSB     REG(0x00, 0xb3)     
#define REG_VS_PIX_STRT_2_LSB     REG(0x00, 0xb4)     
#define REG_VS_LINE_END_2_MSB     REG(0x00, 0xb5)     
#define REG_VS_LINE_END_2_LSB     REG(0x00, 0xb6)     
#define REG_VS_PIX_END_2_MSB      REG(0x00, 0xb7)     
#define REG_VS_PIX_END_2_LSB      REG(0x00, 0xb8)     
#define REG_HS_PIX_START_MSB      REG(0x00, 0xb9)     
#define REG_HS_PIX_START_LSB      REG(0x00, 0xba)     
#define REG_HS_PIX_STOP_MSB       REG(0x00, 0xbb)     
#define REG_HS_PIX_STOP_LSB       REG(0x00, 0xbc)     
#define REG_VWIN_START_1_MSB      REG(0x00, 0xbd)     
#define REG_VWIN_START_1_LSB      REG(0x00, 0xbe)     
#define REG_VWIN_END_1_MSB        REG(0x00, 0xbf)     
#define REG_VWIN_END_1_LSB        REG(0x00, 0xc0)     
#define REG_VWIN_START_2_MSB      REG(0x00, 0xc1)     
#define REG_VWIN_START_2_LSB      REG(0x00, 0xc2)     
#define REG_VWIN_END_2_MSB        REG(0x00, 0xc3)     
#define REG_VWIN_END_2_LSB        REG(0x00, 0xc4)     
#define REG_DE_START_MSB          REG(0x00, 0xc5)     
#define REG_DE_START_LSB          REG(0x00, 0xc6)     
#define REG_DE_STOP_MSB           REG(0x00, 0xc7)     
#define REG_DE_STOP_LSB           REG(0x00, 0xc8)     
#define REG_TBG_CNTRL_0           REG(0x00, 0xca)     
# define TBG_CNTRL_0_TOP_TGL      (1 << 0)
# define TBG_CNTRL_0_TOP_SEL      (1 << 1)
# define TBG_CNTRL_0_DE_EXT       (1 << 2)
# define TBG_CNTRL_0_TOP_EXT      (1 << 3)
# define TBG_CNTRL_0_FRAME_DIS    (1 << 5)
# define TBG_CNTRL_0_SYNC_MTHD    (1 << 6)
# define TBG_CNTRL_0_SYNC_ONCE    (1 << 7)
#define REG_TBG_CNTRL_1           REG(0x00, 0xcb)     
# define TBG_CNTRL_1_H_TGL        (1 << 0)
# define TBG_CNTRL_1_V_TGL        (1 << 1)
# define TBG_CNTRL_1_TGL_EN       (1 << 2)
# define TBG_CNTRL_1_X_EXT        (1 << 3)
# define TBG_CNTRL_1_H_EXT        (1 << 4)
# define TBG_CNTRL_1_V_EXT        (1 << 5)
# define TBG_CNTRL_1_DWIN_DIS     (1 << 6)
#define REG_ENABLE_SPACE          REG(0x00, 0xd6)     
#define REG_HVF_CNTRL_0           REG(0x00, 0xe4)     
# define HVF_CNTRL_0_SM           (1 << 7)
# define HVF_CNTRL_0_RWB          (1 << 6)
# define HVF_CNTRL_0_PREFIL(x)    (((x) & 3) << 2)
# define HVF_CNTRL_0_INTPOL(x)    (((x) & 3) << 0)
#define REG_HVF_CNTRL_1           REG(0x00, 0xe5)     
# define HVF_CNTRL_1_FOR          (1 << 0)
# define HVF_CNTRL_1_YUVBLK       (1 << 1)
# define HVF_CNTRL_1_VQR(x)       (((x) & 3) << 2)
# define HVF_CNTRL_1_PAD(x)       (((x) & 3) << 4)
# define HVF_CNTRL_1_SEMI_PLANAR  (1 << 6)
#define REG_RPT_CNTRL             REG(0x00, 0xf0)     
# define RPT_CNTRL_REPEAT(x)      ((x) & 15)
#define REG_I2S_FORMAT            REG(0x00, 0xfc)     
# define I2S_FORMAT_PHILIPS       (0 << 0)
# define I2S_FORMAT_LEFT_J        (2 << 0)
# define I2S_FORMAT_RIGHT_J       (3 << 0)
#define REG_AIP_CLKSEL            REG(0x00, 0xfd)     
# define AIP_CLKSEL_AIP_SPDIF     (0 << 3)
# define AIP_CLKSEL_AIP_I2S   (1 << 3)
# define AIP_CLKSEL_FS_ACLK   (0 << 0)
# define AIP_CLKSEL_FS_MCLK   (1 << 0)
# define AIP_CLKSEL_FS_FS64SPDIF  (2 << 0)

#define REG_PLL_SERIAL_1          REG(0x02, 0x00)     
# define PLL_SERIAL_1_SRL_FDN     (1 << 0)
# define PLL_SERIAL_1_SRL_IZ(x)   (((x) & 3) << 1)
# define PLL_SERIAL_1_SRL_MAN_IZ  (1 << 6)
#define REG_PLL_SERIAL_2          REG(0x02, 0x01)     
# define PLL_SERIAL_2_SRL_NOSC(x) ((x) << 0)
# define PLL_SERIAL_2_SRL_PR(x)   (((x) & 0xf) << 4)
#define REG_PLL_SERIAL_3          REG(0x02, 0x02)     
# define PLL_SERIAL_3_SRL_CCIR    (1 << 0)
# define PLL_SERIAL_3_SRL_DE      (1 << 2)
# define PLL_SERIAL_3_SRL_PXIN_SEL (1 << 4)
#define REG_SERIALIZER            REG(0x02, 0x03)     
#define REG_BUFFER_OUT            REG(0x02, 0x04)     
#define REG_PLL_SCG1              REG(0x02, 0x05)     
#define REG_PLL_SCG2              REG(0x02, 0x06)     
#define REG_PLL_SCGN1             REG(0x02, 0x07)     
#define REG_PLL_SCGN2             REG(0x02, 0x08)     
#define REG_PLL_SCGR1             REG(0x02, 0x09)     
#define REG_PLL_SCGR2             REG(0x02, 0x0a)     
#define REG_CCIR_DIV              REG(0x02, 0x0c)     
# define CCIR_DIV_REFDIV2        (1 << 0)             
#define REG_AUDIO_DIV             REG(0x02, 0x0e)     
# define AUDIO_DIV_SERCLK_1       0
# define AUDIO_DIV_SERCLK_2       1
# define AUDIO_DIV_SERCLK_4       2
# define AUDIO_DIV_SERCLK_8       3
# define AUDIO_DIV_SERCLK_16      4
# define AUDIO_DIV_SERCLK_32      5
#define REG_SEL_CLK               REG(0x02, 0x11)     
# define SEL_CLK_SEL_CLK1         (1 << 0)
# define SEL_CLK_SEL_VRF_CLK(x)   (((x) & 3) << 1)
# define SEL_CLK_ENA_SC_CLK       (1 << 3)
#define REG_ANA_GENERAL           REG(0x02, 0x12)     

#define REG_EDID_DATA_0           REG(0x09, 0x00)     

#define REG_EDID_CTRL             REG(0x09, 0xfa)     
#define REG_DDC_ADDR              REG(0x09, 0xfb)     
#define REG_DDC_OFFS              REG(0x09, 0xfc)     
#define REG_DDC_SEGM_ADDR         REG(0x09, 0xfd)     
#define REG_DDC_SEGM              REG(0x09, 0xfe)     

#define REG_IF1_HB0               REG(0x10, 0x20)     
#define REG_IF2_HB0               REG(0x10, 0x40)     
#define REG_IF3_HB0               REG(0x10, 0x60)     
#define REG_IF4_HB0               REG(0x10, 0x80)     
#define REG_IF5_HB0               REG(0x10, 0xa0)     

#define HDMI_INFOFRAME_TYPE_AVI   0x82U
#define HDMI_AVI_VERSION          0x02U
#define HDMI_AVI_LENGTH           13U
#define HDMI_AVI_RGB_QUANT_FULL   (2U << 2)

#define REG_AIP_CNTRL_0           REG(0x11, 0x00)     
# define AIP_CNTRL_0_RST_FIFO     (1 << 0)
# define AIP_CNTRL_0_SWAP         (1 << 1)
# define AIP_CNTRL_0_LAYOUT       (1 << 2)
# define AIP_CNTRL_0_ACR_MAN      (1 << 5)
# define AIP_CNTRL_0_RST_CTS      (1 << 6)
#define REG_CA_I2S                REG(0x11, 0x01)     
# define CA_I2S_CA_I2S(x)         (((x) & 31) << 0)
# define CA_I2S_HBR_CHSTAT        (1 << 6)
#define REG_LATENCY_RD            REG(0x11, 0x04)     
#define REG_ACR_CTS_0             REG(0x11, 0x05)     
#define REG_ACR_CTS_1             REG(0x11, 0x06)     
#define REG_ACR_CTS_2             REG(0x11, 0x07)     
#define REG_ACR_N_0               REG(0x11, 0x08)     
#define REG_ACR_N_1               REG(0x11, 0x09)     
#define REG_ACR_N_2               REG(0x11, 0x0a)     
#define REG_CTS_N                 REG(0x11, 0x0c)     
# define CTS_N_K(x)               (((x) & 7) << 0)
# define CTS_N_M(x)               (((x) & 3) << 4)
#define REG_ENC_CNTRL             REG(0x11, 0x0d)     
# define ENC_CNTRL_RST_ENC        (1 << 0)
# define ENC_CNTRL_RST_SEL        (1 << 1)
# define ENC_CNTRL_CTL_CODE(x)    (((x) & 3) << 2)
#define REG_DIP_FLAGS             REG(0x11, 0x0e)     
# define DIP_FLAGS_ACR            (1 << 0)
# define DIP_FLAGS_GC             (1 << 1)
#define REG_DIP_IF_FLAGS          REG(0x11, 0x0f)     
# define DIP_IF_FLAGS_IF1         (1 << 1)
# define DIP_IF_FLAGS_IF2         (1 << 2)
# define DIP_IF_FLAGS_IF3         (1 << 3)
# define DIP_IF_FLAGS_IF4         (1 << 4)
# define DIP_IF_FLAGS_IF5         (1 << 5)
#define REG_CH_STAT_B(x)          REG(0x11, 0x14 + (x)) 

#define REG_TX3                   REG(0x12, 0x9a)     
#define REG_TX4                   REG(0x12, 0x9b)     
# define TX4_PD_RAM               (1 << 1)
#define REG_TX33                  REG(0x12, 0xb8)     
# define TX33_HDMI                (1 << 1)

#define REG_CEC_INTSTATUS     0xee            
# define CEC_INTSTATUS_CEC    (1 << 0)
# define CEC_INTSTATUS_HDMI   (1 << 1)
#define REG_CEC_CAL_XOSC_CTRL1    0xf2
# define CEC_CAL_XOSC_CTRL1_ENA_CAL BIT(0)
#define REG_CEC_DES_FREQ2         0xf5
# define CEC_DES_FREQ2_DIS_AUTOCAL BIT(7)
#define REG_CEC_CLK               0xf6
# define CEC_CLK_FRO              0x11
#define REG_CEC_FRO_IM_CLK_CTRL   0xfb                
# define CEC_FRO_IM_CLK_CTRL_GHOST_DIS (1 << 7)
# define CEC_FRO_IM_CLK_CTRL_ENA_OTP   (1 << 6)
# define CEC_FRO_IM_CLK_CTRL_IMCLK_SEL (1 << 1)
# define CEC_FRO_IM_CLK_CTRL_FRO_DIV   (1 << 0)
#define REG_CEC_RXSHPDINTENA      0xfc            
#define REG_CEC_RXSHPDINT     0xfd            
# define CEC_RXSHPDINT_RXSENS     BIT(0)
# define CEC_RXSHPDINT_HPD        BIT(1)
#define REG_CEC_RXSHPDLEV         0xfe                
# define CEC_RXSHPDLEV_RXSENS     (1 << 0)
# define CEC_RXSHPDLEV_HPD        (1 << 1)

#define REG_CEC_ENAMODS           0xff                
# define CEC_ENAMODS_EN_CEC_CLK   (1 << 7)
# define CEC_ENAMODS_DIS_FRO      (1 << 6)
# define CEC_ENAMODS_DIS_CCLK     (1 << 5)
# define CEC_ENAMODS_EN_RXSENS    (1 << 2)
# define CEC_ENAMODS_EN_HDMI      (1 << 1)
# define CEC_ENAMODS_EN_CEC       (1 << 0)

void tda19988_testmode( void ) {

    uint8_t master_test[3];

    master_test[0] = 0xFF;
    master_test[1] = 0x87;
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_TDA19988_CEC, master_test, 2, 1000);

    master_test[0] = 0xFF;
    master_test[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_TDA19988_HDMI, master_test, 2, 1000);

    master_test[0] = 0xA0;
    master_test[1] = 0x07;    
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_TDA19988_HDMI, master_test, 2, 1000);
    
    master_test[0] = 0xE4;
    master_test[1] = 0xC0;  
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_TDA19988_HDMI, master_test, 2, 1000);

    master_test[0] = 0xF0;
    master_test[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_TDA19988_HDMI, master_test, 2, 1000);

}

void w_reg(uint16_t reg, uint8_t val) {

    uint8_t buf[2];
    buf[0] = REG_CURPAGE;
    buf[1] = REG2PAGE(reg);
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_TDA, buf, 2, 1000);

    buf[0] = REG2ADDR(reg);
    buf[1] = val;
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_TDA, buf, 2, 1000);

}

void w16_reg(uint16_t reg, uint16_t val) {
    uint8_t buf[2];
    buf[0] = REG_CURPAGE;
    buf[1] = REG2PAGE(reg);
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_TDA, buf, 2, 1000);

    buf[0] = REG2ADDR(reg);
    buf[1] = val>>8;
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_TDA, buf, 2, 1000);

    buf[0] = REG2ADDR(reg+1);
    buf[1] = val;
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_TDA, buf, 2, 1000);
}

uint8_t r_reg(uint16_t reg) {

    uint8_t buf[2];
    buf[0] = REG_CURPAGE;
    buf[1] = REG2PAGE(reg);
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_TDA, buf, 2, 1000);

    buf[0] = REG2ADDR(reg);
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_TDA, buf, 1, 1000);
    HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRESS_TDA, buf, 1, 1000);

    return buf[0];
}

uint8_t EDID[256];

uint16_t r_reg_range(uint16_t reg, uint8_t *buf, uint16_t cnt) {

    uint8_t tmp[2];
    tmp[0] = REG_CURPAGE;
    tmp[1] = REG2PAGE(reg);
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_TDA, tmp, 2, 1000);

    tmp[0] = REG2ADDR(reg);
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_TDA, tmp, 1, 1000);
    HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRESS_TDA, buf, cnt, 1000);

    return buf[0];
}

void s_reg(uint16_t reg, uint8_t val){

    uint8_t old_val;

    old_val = r_reg(reg);
    old_val |= val;
    w_reg(reg, old_val);
}

void c_reg(uint16_t reg, uint8_t val){

    uint8_t old_val;

    old_val = r_reg(reg);
    old_val &= ~val;
    w_reg(reg, old_val);
}

static void tda_write_avi_infoframe(void)
{
    const LTDCSYNC_t *mode = &LTDCSYNC[LTDC_VID_FORMAT];
    uint8_t frame[4U + HDMI_AVI_LENGTH] = {0};
    uint8_t checksum = 0U;
    uint32_t i;

    c_reg(REG_DIP_IF_FLAGS, DIP_IF_FLAGS_IF2);

    if (mode->hdmi_vic == 0U)
        return;

    frame[0] = HDMI_INFOFRAME_TYPE_AVI;
    frame[1] = HDMI_AVI_VERSION;
    frame[2] = HDMI_AVI_LENGTH;
    frame[3] = 0U;
    frame[4] = 0x02U;
    frame[5] = (uint8_t)((mode->hdmi_aspect & 0x03U) << 4);
    frame[6] = HDMI_AVI_RGB_QUANT_FULL;
    frame[7] = (uint8_t)(mode->hdmi_vic & 0x7fU);
    frame[8] = 0U;

    for (i = 0U; i < (uint32_t)sizeof(frame); ++i)
        checksum = (uint8_t)(checksum + frame[i]);
    frame[3] = (uint8_t)(0U - checksum);

    for (i = 0U; i < (uint32_t)sizeof(frame); ++i)
        w_reg((uint16_t)(REG_IF2_HB0 + i), frame[i]);

    s_reg(REG_DIP_IF_FLAGS, DIP_IF_FLAGS_IF2);
}

void read_edid(void) {
    w_reg(0x00F9, 0x00);
    w_reg(0x00FE, 0xa0);

    HAL_I2C_Mem_Read(&hi2c1, 0xa0, 0, I2C_MEMADD_SIZE_8BIT, &EDID[0], 256, 1000);
}

uint8_t debug[200];
extern LTDC_HandleTypeDef hltdc;

void tda_init(void) {
    w_reg(REG_SOFTRESET, SOFTRESET_AUDIO | SOFTRESET_I2C_MASTER);
    HAL_Delay(50);
    w_reg(REG_SOFTRESET, 0);
    HAL_Delay(50);

    s_reg(REG_MAIN_CNTRL0, MAIN_CNTRL0_SR);
    c_reg(REG_MAIN_CNTRL0, MAIN_CNTRL0_SR);
#if 0
    w_reg(REG_PLL_SERIAL_1, 0x00);
    w_reg(REG_PLL_SERIAL_2, PLL_SERIAL_2_SRL_NOSC(1));
    w_reg(REG_PLL_SERIAL_3, 0x00);
    w_reg(REG_SERIALIZER,   0x00);
    w_reg(REG_BUFFER_OUT,   0x00);
    w_reg(REG_PLL_SCG1,     0x00);
    w_reg(REG_AUDIO_DIV,    AUDIO_DIV_SERCLK_8);
    w_reg(REG_SEL_CLK,      SEL_CLK_SEL_CLK1 | SEL_CLK_ENA_SC_CLK);
    w_reg(REG_PLL_SCGN1,    0xfa);
    w_reg(REG_PLL_SCGN2,    0x00);
    w_reg(REG_PLL_SCGR1,    0x5b);
    w_reg(REG_PLL_SCGR2,    0x00);
    w_reg(REG_PLL_SCG2,     0x10);
    w_reg(REG_MUX_VP_VIP_OUT, 0x24);
#endif

    read_edid();

    uint32_t pixel_clock;
    uint8_t reg, rep, div, sel_clk;
    uint16_t ref_pix, ref_line, n_pix, n_line;
    uint16_t hs_pix_s, hs_pix_e;
    uint16_t vs1_pix_s, vs1_pix_e, vs1_line_s, vs1_line_e;
    uint16_t vs2_pix_s, vs2_pix_e, vs2_line_s, vs2_line_e;
    uint16_t vwin1_line_s, vwin1_line_e;
    uint16_t vwin2_line_s, vwin2_line_e;
    uint16_t de_pix_s, de_pix_e;

    rep = 0;
    sel_clk = SEL_CLK_ENA_SC_CLK | SEL_CLK_SEL_VRF_CLK(0);
    pixel_clock = (LTDCSYNC[LTDC_VID_FORMAT].pll3n/LTDCSYNC[LTDC_VID_FORMAT].pll3r) * (1 + rep);

    for (div = 0; div < 3; div++)
        if (80 >> div <= pixel_clock)
            break;

    w_reg(REG_ENA_VP_0, 0);
    w_reg(REG_ENA_VP_1, 0);
    w_reg(REG_ENA_VP_2, 0);
    w_reg(REG_ENA_AP, 0);
    s_reg(REG_AIP_CNTRL_0, AIP_CNTRL_0_RST_FIFO);
    w_reg(REG_TBG_CNTRL_1, TBG_CNTRL_1_DWIN_DIS);
    c_reg(REG_TX33, TX33_HDMI);
    w_reg(REG_ENC_CNTRL, ENC_CNTRL_CTL_CODE(0));

    c_reg(REG_MAIN_CNTRL0, MAIN_CNTRL0_SCALER);
    w_reg(REG_HVF_CNTRL_0, HVF_CNTRL_0_PREFIL(0) | HVF_CNTRL_0_INTPOL(0));
    s_reg(REG_FEAT_POWERDOWN,FEAT_POWERDOWN_PREFILT);
    w_reg(REG_VIP_CNTRL_5, VIP_CNTRL_5_SP_CNT(0));
    w_reg(REG_VIP_CNTRL_4, VIP_CNTRL_4_BLANKIT(0) | VIP_CNTRL_4_BLC(0));

    c_reg(REG_PLL_SERIAL_1, PLL_SERIAL_1_SRL_MAN_IZ);
    c_reg(REG_PLL_SERIAL_3, PLL_SERIAL_3_SRL_CCIR | PLL_SERIAL_3_SRL_DE);
    w_reg(REG_SERIALIZER, 0);
    w_reg(REG_HVF_CNTRL_1, HVF_CNTRL_1_VQR(0));

    w_reg(REG_RPT_CNTRL, RPT_CNTRL_REPEAT(rep));
    w_reg(REG_CCIR_DIV, 0x00);
    w_reg(REG_SEL_CLK, sel_clk);
    w_reg(REG_PLL_SERIAL_2, PLL_SERIAL_2_SRL_NOSC(div) | PLL_SERIAL_2_SRL_PR(rep));

    w_reg(REG_MAT_CONTRL, MAT_CONTRL_MAT_BP | MAT_CONTRL_MAT_SC(1));
    s_reg(REG_FEAT_POWERDOWN, FEAT_POWERDOWN_CSC);
    w_reg(REG_ANA_GENERAL, 0x09);

    reg = VIP_CNTRL_3_SYNC_HS;
    if (hltdc.Init.HSPolarity == LTDC_HSPOLARITY_AL)
        reg |= VIP_CNTRL_3_H_TGL;
    if (hltdc.Init.VSPolarity == LTDC_VSPOLARITY_AL)
        reg |= VIP_CNTRL_3_V_TGL;
    w_reg(REG_VIP_CNTRL_3, reg);

    n_pix  = LTDCSYNC[LTDC_VID_FORMAT].hsw + LTDCSYNC[LTDC_VID_FORMAT].ahw + LTDCSYNC[LTDC_VID_FORMAT].hbp + LTDCSYNC[LTDC_VID_FORMAT].hfp;
    n_line = LTDCSYNC[LTDC_VID_FORMAT].vsh + LTDCSYNC[LTDC_VID_FORMAT].avh + LTDCSYNC[LTDC_VID_FORMAT].vbp + LTDCSYNC[LTDC_VID_FORMAT].vfp;
    hs_pix_e = LTDCSYNC[LTDC_VID_FORMAT].hfp + LTDCSYNC[LTDC_VID_FORMAT].hsw;
    hs_pix_s = LTDCSYNC[LTDC_VID_FORMAT].hfp;
    de_pix_e = n_pix;
    de_pix_s = LTDCSYNC[LTDC_VID_FORMAT].hsw + LTDCSYNC[LTDC_VID_FORMAT].hbp + LTDCSYNC[LTDC_VID_FORMAT].hfp;
    ref_pix = 3 + hs_pix_s;
    ref_line = 1 + LTDCSYNC[LTDC_VID_FORMAT].vfp;
    vwin1_line_s = LTDCSYNC[LTDC_VID_FORMAT].vfp + LTDCSYNC[LTDC_VID_FORMAT].vsh + LTDCSYNC[LTDC_VID_FORMAT].vbp - 1;
    vwin1_line_e = vwin1_line_s + LTDCSYNC[LTDC_VID_FORMAT].avh;
    vs1_pix_s = vs1_pix_e = hs_pix_s;
    vs1_line_s = LTDCSYNC[LTDC_VID_FORMAT].vfp;
    vs1_line_e = vs1_line_s + LTDCSYNC[LTDC_VID_FORMAT].vsh;
    vwin2_line_s = vwin2_line_e = 0;
    vs2_pix_s = vs2_pix_e = 0;
    vs2_line_s = vs2_line_e = 0;

    w_reg(REG_VIDFORMAT, 0x00);
    w16_reg(REG_REFPIX_MSB, ref_pix);
    w16_reg(REG_REFLINE_MSB, ref_line);
    w16_reg(REG_NPIX_MSB, n_pix);
    w16_reg(REG_NLINE_MSB, n_line);
    w16_reg(REG_VS_LINE_STRT_1_MSB, vs1_line_s);
    w16_reg(REG_VS_PIX_STRT_1_MSB, vs1_pix_s);
    w16_reg(REG_VS_LINE_END_1_MSB, vs1_line_e);
    w16_reg(REG_VS_PIX_END_1_MSB, vs1_pix_e);
    w16_reg(REG_VS_LINE_STRT_2_MSB, vs2_line_s);
    w16_reg(REG_VS_PIX_STRT_2_MSB, vs2_pix_s);
    w16_reg(REG_VS_LINE_END_2_MSB, vs2_line_e);
    w16_reg(REG_VS_PIX_END_2_MSB, vs2_pix_e);
    w16_reg(REG_HS_PIX_START_MSB, hs_pix_s);
    w16_reg(REG_HS_PIX_STOP_MSB, hs_pix_e);
    w16_reg(REG_VWIN_START_1_MSB, vwin1_line_s);
    w16_reg(REG_VWIN_END_1_MSB, vwin1_line_e);
    w16_reg(REG_VWIN_START_2_MSB, vwin2_line_s);
    w16_reg(REG_VWIN_END_2_MSB, vwin2_line_e);
    w16_reg(REG_DE_START_MSB, de_pix_s);
    w16_reg(REG_DE_STOP_MSB, de_pix_e);

    reg = TBG_CNTRL_1_DWIN_DIS | TBG_CNTRL_1_TGL_EN;
    if (hltdc.Init.HSPolarity == LTDC_HSPOLARITY_AL)
        reg |= TBG_CNTRL_1_H_TGL;
    if (hltdc.Init.VSPolarity == LTDC_VSPOLARITY_AL)
        reg |= TBG_CNTRL_1_V_TGL;
    w_reg(REG_TBG_CNTRL_1, reg);

    w_reg(REG_TBG_CNTRL_0, 0);
    reg &= ~TBG_CNTRL_1_DWIN_DIS;
    w_reg(REG_TBG_CNTRL_1, reg);
    w_reg(REG_ENC_CNTRL, ENC_CNTRL_CTL_CODE(1));
    s_reg(REG_TX33, TX33_HDMI);
    tda_write_avi_infoframe();

    HAL_Delay(400);

    w_reg(REG_ENA_VP_0, 0xff);
    w_reg(REG_ENA_VP_1, 0xff);
    w_reg(REG_ENA_VP_2, 0xff);
    w_reg(REG_VIP_CNTRL_0, VIP_CNTRL_0_SWAP_A(2) | VIP_CNTRL_0_SWAP_B(3));
    w_reg(REG_VIP_CNTRL_1, VIP_CNTRL_1_SWAP_C(4) | VIP_CNTRL_1_SWAP_D(5));
    w_reg(REG_VIP_CNTRL_2, VIP_CNTRL_2_SWAP_E(0) | VIP_CNTRL_2_SWAP_F(1));

    w_reg(0x1100, 1<<6|1<<0);
    HAL_Delay(100);
    w_reg(0x1100, 0);
    w_reg(REG_AIP_CLKSEL, AIP_CLKSEL_AIP_I2S);
}
