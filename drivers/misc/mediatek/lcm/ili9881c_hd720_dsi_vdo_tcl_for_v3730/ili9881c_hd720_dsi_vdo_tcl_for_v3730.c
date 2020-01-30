#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
    #include <platform/mt_gpio.h>
    #include <string.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define AUXADC_LCM_VOLTAGE_CHANNEL     12
#define AUXADC_ADC_FDD_RF_PARAMS_DYNAMIC_CUSTOM_CH_CHANNEL     1
#define MAX_VOLTAGE (1000)     // zhoulidong  add for lcm detect

#define LCM_ID_ILI9881                                    0x9881
#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifdef BUILD_LK
#define LCM_PRINT printf
#else
#if defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif
#endif

#define LCM_DBG(fmt, arg...) \
    LCM_PRINT ("[ili9881c_TCL] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))




#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							0

#define REGFLAG_END_OF_TABLE                                	  0xFD   // END OF REGISTERS MARKER
#define REGFLAG_DELAY                                           0xFC
#define AUXADC_LCM_VOLTAGE_CHANNEL     12

struct LCM_setting_table
{
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static struct LCM_setting_table  lcm_deep_sleep_mode_in_setting_v2[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
};

static struct LCM_setting_table lcm_initialization_setting_v2[] = 
{
	{0xFF,3,{0x98,0x81,0x03}},            
	//GIP_1                               
	{0x01,1,{0x00}},                      
	{0x02,1,{0x00}},                      
	{0x03,1,{0x53}},                      
	{0x04,1,{0x14}},                      
	{0x05,1,{0x00}},                      
	{0x06,1,{0x06}},                      
	{0x07,1,{0x01}},                      
	{0x08,1,{0x00}},                      
	{0x09,1,{0x01}},                      
	{0x0a,1,{0x19}},                      
	{0x0b,1,{0x01}},                      
	{0x0c,1,{0x00}},                      
	{0x0d,1,{0x00}},                      
	{0x0e,1,{0x00}},                      
	{0x0f,1,{0x19}},                      
	{0x10,1,{0x19}},                      
	{0x11,1,{0x00}},                      
	{0x12,1,{0x00}},                      
	{0x13,1,{0x00}},                      
	{0x14,1,{0x00}},                      
	{0x15,1,{0x00}},                      
	{0x16,1,{0x00}},                      
	{0x17,1,{0x00}},                      
	{0x18,1,{0x00}},                      
	{0x19,1,{0x00}},                      
	{0x1a,1,{0x00}},                      
	{0x1b,1,{0x00}},                      
	{0x1c,1,{0x00}},                      
	{0x1d,1,{0x00}},                      
	{0x1e,1,{0x40}},                      
	{0x1f,1,{0x40}},                      
	{0x20,1,{0x02}},                      
	{0x21,1,{0x05}},                      
	{0x22,1,{0x02}},                      
	{0x23,1,{0x00}},                      
	{0x24,1,{0x87}},                      
	{0x25,1,{0x87}},                      
	{0x26,1,{0x00}},                      
	{0x27,1,{0x00}},                      
	{0x28,1,{0x3B}},                      
	{0x29,1,{0x03}},                      
	{0x2a,1,{0x00}},                      
	{0x2b,1,{0x00}},                      
	{0x2c,1,{0x00}},                      
	{0x2d,1,{0x00}},                      
	{0x2e,1,{0x00}},                      
	{0x2f,1,{0x00}},                      
	{0x30,1,{0x00}},                      
	{0x31,1,{0x00}},                      
	{0x32,1,{0x00}},                      
	{0x33,1,{0x00}},                      
	{0x34,1,{0x04}},                      
	{0x35,1,{0x00}},                      
	{0x36,1,{0x00}},                      
	{0x37,1,{0x00}},                      
	{0x38,1,{0x01}},                      
	{0x39,1,{0x01}},                      
	{0x3a,1,{0x40}},                      
	{0x3b,1,{0x40}},                      
	{0x3c,1,{0x00}},                      
	{0x3d,1,{0x00}},                      
	{0x3e,1,{0x00}},                      
	{0x3f,1,{0x00}},                      
	{0x40,1,{0x00}},                      
	{0x41,1,{0x88}},                      
	{0x42,1,{0x00}},                      
	{0x43,1,{0x00}},                      
	{0x44,1,{0x00}},                      
	//GIP_2                               
	{0x50,1,{0x01}},                      
	{0x51,1,{0x23}},                      
	{0x52,1,{0x45}},                      
	{0x53,1,{0x67}},                      
	{0x54,1,{0x89}},                      
	{0x55,1,{0xab}},                      
	{0x56,1,{0x01}},                      
	{0x57,1,{0x23}},                      
	{0x58,1,{0x45}},                      
	{0x59,1,{0x67}},                      
	{0x5a,1,{0x89}},                      
	{0x5b,1,{0xab}},                      
	{0x5c,1,{0xcd}},                      
	{0x5d,1,{0xef}},                      
	//GIP_3                               
	{0x5e,1,{0x11}},                       
	{0x5f,1,{0x08}},                       
	{0x60,1,{0x00}},                       
	{0x61,1,{0x01}},                       
	{0x62,1,{0x02}},                       
	{0x63,1,{0x02}},                       
	{0x64,1,{0x0F}},                       
	{0x65,1,{0x0D}},                       
	{0x66,1,{0x0E}},                       
	{0x67,1,{0x0C}},                       
	{0x68,1,{0x02}},                       
	{0x69,1,{0x02}},                       
	{0x6a,1,{0x02}},                       
	{0x6b,1,{0x02}},                       
	{0x6c,1,{0x02}},                       
	{0x6d,1,{0x02}},                       
	{0x6e,1,{0x06}},                      
	{0x6f,1,{0x02}},                      
	{0x70,1,{0x02}},                      
	{0x71,1,{0x02}},                      
	{0x72,1,{0x02}},                      
	{0x73,1,{0x02}},                      
	{0x74,1,{0x02}},                      
	{0x75,1,{0x06}},                      
	{0x76,1,{0x00}},                      
	{0x77,1,{0x01}},                      
	{0x78,1,{0x02}},                      
	{0x79,1,{0x02}},                      
	{0x7a,1,{0x0F}},                      
	{0x7b,1,{0x0D}},                      
	{0x7c,1,{0x0E}},                      
	{0x7d,1,{0x0C}},                      
	{0x7e,1,{0x02}},                      
	{0x7f,1,{0x02}},                      
	{0x80,1,{0x02}},                      
	{0x81,1,{0x02}},                      
	{0x82,1,{0x02}},                      
	{0x83,1,{0x02}},                      
	{0x84,1,{0x08}},                      
	{0x85,1,{0x02}},                      
	{0x86,1,{0x02}},                      
	{0x87,1,{0x02}},                      
	{0x88,1,{0x02}},                      
	{0x89,1,{0x02}},                      
	{0x8A,1,{0x02}},                      
	//CMD_Page 4                          
	{0xFF,3,{0x98,0x81,0x04}},            
	{0x00,1,{0x00}},                      
	{0x6C,1,{0x15}},                      
	{0x6E,1,{0x2B}},//di_pwr_reg=0 VGH cla
	{0x6F,1,{0x33}},//reg vcl + VGH pumpin
	{0x3A,1,{0xA4}},//POWER SAVING        
	{0x8D,1,{0x15}},//VGL clamp -10.23V   
	{0x87,1,{0xBA}},//ESD                 
	{0x26,1,{0x76}},                      
	{0xB2,1,{0xD1}},                      
	//CMD_Page 1                          
	{0xFF,3,{0x98,0x81,0x01}},            
	{0x22,1,{0x0A}},//BGR                 
	{0x53,1,{0x70}},//VCOM1 Forward scan  
	{0x55,1,{0x73}},//VCOM2 Backword scan 
	{0x50,1,{0xA7}},//VREG1OUT=4.716V     
	{0x51,1,{0xA7}},//VREG2OUT=-4.716V    
	{0x31,1,{0x00}},//column inversion    
	{0x60,1,{0x14}},//SDT                 
		                              
	{0xA0,1,{0x0F}},//VP255Gamma P        
	{0xA1,1,{0x37}},//VP251               
	{0xA2,1,{0x47}},//VP247               
	{0xA3,1,{0x0F}},//VP243               
	{0xA4,1,{0x12}},//VP239               
	{0xA5,1,{0x25}},//VP231               
	{0xA6,1,{0x19}},//VP219               
	{0xA7,1,{0x1D}},//VP203               
	{0xA8,1,{0xAA}},//VP175               
	{0xA9,1,{0x1A}},//VP144               
	{0xAA,1,{0x28}},//VP111               
	{0xAB,1,{0x93}},//VP80                
	{0xAC,1,{0x1D}},//VP52                
	{0xAD,1,{0x1C}},//VP36                
	{0xAE,1,{0x50}},//VP24                
	{0xAF,1,{0x24}},//VP16                
	{0xB0,1,{0x2A}},//VP12                
	{0xB1,1,{0x57}},//VP8                 
	{0xB2,1,{0x65}},//VP4                 
	{0xB3,1,{0x39}},//VP0                 
		                              
	{0xC0,1,{0x00}},//VN255 GAMMA N       
	{0xC1,1,{0x1D}},//VN251               
	{0xC2,1,{0x29}},//VN247               
	{0xC3,1,{0x16}},//VN243               
	{0xC4,1,{0x19}},//VN239               
	{0xC5,1,{0x2D}},//VN231               
	{0xC6,1,{0x1E}},//VN219               
	{0xC7,1,{0x1E}},//VN203               
	{0xC8,1,{0x91}},//VN175               
	{0xC9,1,{0x1C}},//VN144               
	{0xCA,1,{0x29}},//VN111               
	{0xCB,1,{0x85}},//VN80                
	{0xCC,1,{0x1E}},//VN52                
	{0xCD,1,{0x1D}},//VN36                
	{0xCE,1,{0x4F}},//VN24                
	{0xCF,1,{0x25}},//VN16                
	{0xD0,1,{0x29}},//VN12                
	{0xD1,1,{0x4E}},//VN8                 
	{0xD2,1,{0x5C}},//VN4                 
	{0xD3,1,{0x39}},//VN0                 
	//CMD_Page 0                          
	{0xFF,3,{0x98,0x81,0x00}},            
	{0x11,1,{0x00}},	//sleep out Dela
		                              
	{REGFLAG_DELAY,120,{}},             
	{0x29,1,{0x00}},	//display on    
	{0x35,1,{0x00}},	//TE on         
	{REGFLAG_DELAY,200,{}},               
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;
    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;
        switch (cmd)
        {
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
            case REGFLAG_END_OF_TABLE :
                break;
            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }

}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

	

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_THREE_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		

    params->dsi.vertical_sync_active                =8;//6;
    params->dsi.vertical_backporch                   =6; //6;
    params->dsi.vertical_frontporch                   =4;//6;
		params->dsi.vertical_active_line = FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active                = 26;//16;///////////////20 20 4  20  14  6
    params->dsi.horizontal_backporch                = 24;//15;
    params->dsi.horizontal_frontporch                = 38;//6;
		params->dsi.horizontal_active_pixel = FRAME_WIDTH;

		// Bit rate calculation
		//params->dsi.PLL_CLOCK=205;
		params->dsi.PLL_CLOCK=285; 
		params->dsi.ssc_disable=1;
		
		//yixuhong 20150511 add esd check function
#ifndef BUILD_LK	
	params->dsi.esd_check_enable = 1; 
	params->dsi.customization_esd_check_enable = 1;//0:te esd check 1:read register
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	
	//params->dsi.lcm_esd_check_table[1].cmd = 0x0B;
//	params->dsi.lcm_esd_check_table[1].count = 1;
//	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
	params->dsi.lcm_esd_check_table[1].cmd = 0x0D;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
#endif /*BUILD_LK*/

}

static void lcm_init(void)
{
	LCM_DBG(); 
	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(10); 
	SET_RESET_PIN(1);
	MDELAY(120);
	push_table(lcm_initialization_setting_v2, sizeof(lcm_initialization_setting_v2) / sizeof(struct LCM_setting_table), 1);
	LCM_DBG("jacy debug,lcm init end \n");
}

static void lcm_suspend(void)
{	
	LCM_DBG();
    push_table(lcm_deep_sleep_mode_in_setting_v2, sizeof(lcm_deep_sleep_mode_in_setting_v2) / sizeof(struct LCM_setting_table), 1);
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(20); // 1ms

    SET_RESET_PIN(1);
    MDELAY(120);
}

static void lcm_resume(void)
{
	LCM_DBG();
	SET_RESET_PIN(0);
	MDELAY(10); 
	SET_RESET_PIN(1);
	MDELAY(120); 
	push_table(lcm_initialization_setting_v2, sizeof(lcm_initialization_setting_v2) / sizeof(struct LCM_setting_table), 1);
}      

static unsigned int lcm_compare_id(void)
{
     int data[4] = {0,0,0,0};
    int res = 0;
    int rawdata = 0;
    int lcm_vol = 0;
	
	int array[4];
	char buffer[4]={0,0,0,0};
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	
	array[0]= 0x00043902;
	array[1]= 0x018198FF;
	dsi_set_cmdq(&array, 2, 1);
	
	array[0] = 0x00023700;// read id return 2 bytes,version and id
	dsi_set_cmdq(array, 1, 1);
	MDELAY(1);	
	read_reg_v2(0x00, buffer, 2);
	id_high = buffer[0];

	array[0] = 0x00023700;// read id return 2 bytes,version and id
	dsi_set_cmdq(array, 1, 1);
	MDELAY(1);	
	read_reg_v2(0x01, buffer, 2);  
	id_low = buffer[0];
	
	id = (id_high<<8) | id_low;

	
#ifdef BUILD_LK
	printf(" ILI9881 TXD LK read id=0x%x, id1=0x%x, id2=0x%x",id, id_high,id_low);
#else
        printk(" ILI9881 TXD LK read id=0x%x, id1=0x%x, id2=0x%x",id, id_high,id_low);
#endif 

#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
    res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
    if(res < 0)
    {
   #ifdef BUILD_LK 
    printf("read adc error");
   #else 
   printk("read adc error");
   #endif
    return 0;

    }
    lcm_vol = data[0]*1000+data[1]*10;
#endif
    
#ifdef BUILD_LK
   	printf("[adc_uboot]: lcm_vol= %d\n",lcm_vol);
#else
    printk("[adc_uboot]: lcm_vol= %d\n",lcm_vol);
#endif
    if ((lcm_vol < MAX_VOLTAGE )&&(LCM_ID_ILI9881 == id))
    {
    return 1;
    }
    return 0;
}

LCM_DRIVER ili9881c_hd720_dsi_vdo_tcl_for_v3730_lcm_drv = 
{
	.name		= "ili9881c_hd720_dsi_vdo_tcl",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};
