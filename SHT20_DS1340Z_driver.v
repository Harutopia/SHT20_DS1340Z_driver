// --------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<
// --------------------------------------------------------------------
//模块说明
//此模块由小脚丫FPGA扩展底板 实验9 与 实验10 中的IIC驱动模块合并而成
//版权归原作者所有
//此模块仅供学习参考
// SHT20 温度驱动模块：
// --------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<
// --------------------------------------------------------------------
// Module: SHT20_Driver
// 
// Author: Step
// 
// Description: SHT20_Driver
// 
// Web: www.stepfpga.com
//
// --------------------------------------------------------------------
// Code Revision History :
// --------------------------------------------------------------------
// Version: |Mod. Date:   |Changes Made:
// V1.1     |2016/10/30   |Initial ver
// --------------------------------------------------------------------
//
//DS1340Z 时钟模块
//// --------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<
// --------------------------------------------------------------------
// Module: DS1340Z_driver
// 
// Author: Step
// 
// Description: DS1340Z_driver
// 
// Web: www.stepfpga.com
//
// --------------------------------------------------------------------
// Code Revision History :
// --------------------------------------------------------------------
// Version: |Mod. Date:   |Changes Made:
// V1.1     |2016/10/30   |Initial ver
// --------------------------------------------------------------------
module SHT20_DS1340Z_driver
(
input				clk,		//系统时钟
input				rst_n,	//系统复位，低有效

input				key_set,	//按动脉冲输入

output				i2c_scl,	//I2C总线SCL
inout				i2c_sda,	//I2C总线SDA

input		[7:0]	adj_sec,	//秒钟调整输入
input		[7:0]	adj_min,    //分钟调整输入
input		[7:0]	adj_hour,   //时钟调整输入
input		[7:0]	adj_week,   //星期调整输入
input		[7:0]	adj_day,    //日期调整输入
input		[7:0]	adj_mon,    //月份调整输入
input		[7:0]	adj_year,   //年份调整输入

output	reg	[7:0]	rtc_sec,	//实时秒钟输出
output	reg	[7:0]	rtc_min,    //实时分钟输出
output	reg	[7:0]	rtc_hour,   //实时时钟输出
output	reg	[7:0]	rtc_week,   //实时星期输出
output	reg	[7:0]	rtc_day,    //实时日期输出
output	reg	[7:0]	rtc_mon,    //实时月份输出
output	reg	[7:0]	rtc_year,   //实时年份输出

output	reg	[15:0]	T_code,		//温度码值
output	reg	[15:0]	H_code		//湿度码值
);

parameter	CNT_NUM	=	15;
//ds1340
localparam	IDLE		=	4'd0;
localparam	MAIN		=	4'd1;
localparam	START		=	4'd2;
localparam	WRITE		=	4'd3;
localparam	READ		=	4'd4;
localparam	STOP		=	4'd5;
//sht20
localparam	SHT_MODE1	=	4'd6;
localparam	SHT_MODE2	=	4'd7;
localparam	SHT_START	=	4'd8;
localparam	SHT_WRITE	=	4'd9;
localparam	SHT_READ	=	4'd10;
localparam	SHT_STOP	=	4'd11;
localparam	SHT_DELAY	=	4'd12;

localparam	ACK		=	1'b0;
localparam	NACK	=	1'b1;

//使用计数器分频产生400KHz时钟信号clk_400khz
reg					clk_400khz;
reg		[9:0]		cnt_400khz;
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		cnt_400khz <= 10'd0;
		clk_400khz <= 1'b0;
	end else if(cnt_400khz >= CNT_NUM-1) begin
		cnt_400khz <= 10'd0;
		clk_400khz <= ~clk_400khz;
	end else begin
		cnt_400khz <= cnt_400khz + 1'b1;
	end
end

reg					set_flag;

always@(posedge clk or negedge rst_n) begin
	if(!rst_n) set_flag <= 1'b0;
	else if(cnt_main==5'd11) set_flag <= 1'b0;	//当完成写入时间操作后复位set_flag
	else if(key_set) set_flag <= 1'b1;			//按键脉冲控制set_flag置位
	else set_flag <= set_flag;
end
//公共
reg					ack;
reg					scl_out_r;
reg					sda_out_r;
reg		[3:0] 		state;

reg		[3:0]		cnt;
reg		[5:0]		cnt_main;
reg		[3:0]		cnt_start;
reg		[3:0]		cnt_write;
reg		[4:0]		cnt_read;
reg		[3:0]		cnt_stop;

reg		[7:0]		data_wr;

//ds1340
reg		[7:0]		rtc_data_r;

//sht20
reg ack_flag;
reg [3:0] cnt_mode1, cnt_mode2;
reg [7:0] dev_addr, reg_addr, reg_data, data_r, dat_l, dat_h;
reg [23:0] cnt_delay, num_delay;
reg [3:0]  state_back;


always@(posedge clk_400khz or negedge rst_n) begin
	if(!rst_n) begin	//如果按键复位，将相关数据初始化
		scl_out_r <= 1'd1;
		sda_out_r <= 1'd1;
		ack <= ACK;
		cnt <= 1'b0;
		cnt_main <= 6'd12;
		cnt_start <= 3'd0;
		cnt_write <= 3'd0;
		cnt_read <= 5'd0;
		cnt_stop <= 1'd0;
		state <= IDLE;
		//sht20
		ack_flag <= 1'b0;
		cnt_mode1 <= 1'b0;
		cnt_mode2 <= 1'b0;
		cnt_delay <= 1'b0;
		num_delay <= 24'd48000;
		state_back <= IDLE;
	end else begin
		case(state)
			IDLE:begin	//软件自复位，主要用于程序跑飞后的处理
					scl_out_r <= 1'd1;
					sda_out_r <= 1'd1;
					ack <= ACK;
					cnt <= 1'b0;
					cnt_main <= 6'd12;
					cnt_start <= 3'd0;
					cnt_write <= 3'd0;
					cnt_read <= 5'd0;
					cnt_stop <= 1'd0;
					state <= MAIN;
					//sht20 plus
					ack_flag <= 1'b0;
					cnt_mode1 <= 1'b0;
					cnt_mode2 <= 1'b0;
					cnt_delay <= 1'b0;
					num_delay <= 24'd48000;
					state_back <= IDLE;
				end
			MAIN:begin
					if(cnt_main >= 6'd42) //对MAIN中的子状态执行控制cnt_main
						if(set_flag)cnt_main <= 6'd0;	//当set_flag被置位时才会执行时间写入操作
						else cnt_main <= 6'd12;  		//否则只执行时间读取操作
					else cnt_main <= cnt_main + 1'b1;	
					case(cnt_main)
						6'd0:	begin state <= START; end	//I2C通信时序中的START
						6'd1:	begin data_wr <= 8'hd0; state <= WRITE; end		//写地址为8'hd0
						6'd2:	begin data_wr <= 8'h00; state <= WRITE; end		//8'h00，寄存器初始地址
						6'd3:	begin data_wr <= adj_sec; state <= WRITE; end	//写秒
						6'd4:	begin data_wr <= adj_min; state <= WRITE; end	//写分
						6'd5:	begin data_wr <= adj_hour; state <= WRITE; end	//写时
						6'd6:	begin data_wr <= adj_week; state <= WRITE; end	//写周
						6'd7:	begin data_wr <= adj_day; state <= WRITE; end	//写日
						6'd8:	begin data_wr <= adj_mon; state <= WRITE; end	//写月
						6'd9:	begin data_wr <= adj_year; state <= WRITE; end	//写年
						6'd10:	begin data_wr <= 8'h40; state <= WRITE; end		//8'h40，控制
						6'd11:	begin state <= STOP; end	//I2C通信时序中的STOP
						
						6'd12:	begin state <= START; end	//I2C通信时序中的START
						6'd13:	begin data_wr <= 8'hd0; state <= WRITE; end	//写地址为8'hd0
						6'd14:	begin data_wr <= 8'h00; state <= WRITE; end	//8'h00，寄存器初始地址
						6'd15:	begin state <= START; end	//I2C通信时序中的START
						6'd16:	begin data_wr <= 8'hd1; state <= WRITE; end	//读地址为8'hd1
						6'd17:	begin ack <= ACK; state <= READ; end	//读秒
						6'd18:	begin rtc_sec <= rtc_data_r; end
						6'd19:	begin ack <= ACK; state <= READ; end	//读分
						6'd20:	begin rtc_min <= rtc_data_r; end
						6'd21:	begin ack <= ACK; state <= READ; end	//读时
						6'd22:	begin rtc_hour <= rtc_data_r; end
						6'd23:	begin ack <= ACK; state <= READ; end	//读周
						6'd24:	begin rtc_week <= rtc_data_r; end
						6'd25:	begin ack <= ACK; state <= READ; end	//读日
						6'd26:	begin rtc_day <= rtc_data_r; end
						6'd27:	begin ack <= ACK; state <= READ; end	//读月
						6'd28:	begin rtc_mon <= rtc_data_r; end
						6'd29:	begin ack <= ACK; state <= READ; end	//读年
						6'd30:	begin rtc_year <= rtc_data_r; end
						6'd31:	begin ack <= NACK; state <= READ; end	//控制
						6'd32:	begin state <= STOP; end	//I2C通信时序中的STOP，读取完成标志

						//sht20
						6'd33:	begin dev_addr <= 7'h40; reg_addr <= 8'hfe; state <= SHT_MODE1; end	//软件复位
						6'd34:	begin num_delay <= 24'd6000; state <= SHT_DELAY; end	//15ms延时
							
						6'd35:	begin dev_addr <= 7'h40; reg_addr <= 8'hf3; state <= SHT_MODE1; end	//写入配置
						6'd36:	begin num_delay <= 24'd34000; state <= SHT_DELAY; end	//85ms延时
						6'd37:	begin dev_addr <= 7'h40; state <= SHT_MODE2; end	//读取配置
						6'd38:	begin T_code <= {dat_h,dat_l}; end	//读取数据
							
						6'd39:	begin dev_addr <= 7'h40; reg_addr <= 8'hf5; state <= SHT_MODE1; end	//写入配置
						6'd40:	begin num_delay <= 24'd12000; state <= SHT_DELAY; end	//30ms延时
						6'd41:	begin dev_addr <= 7'h40; state <= SHT_MODE2; end	//读取配置
						6'd42:	begin H_code <= {dat_h,dat_l}; end	//读取数据
						default: state <= IDLE;	//如果程序失控，进入IDLE自复位状态
					endcase
				end
			START:begin	//I2C通信时序中的起始START
					if(cnt_start >= 3'd5) cnt_start <= 1'b0;	//对START中的子状态执行控制cnt_start
					else cnt_start <= cnt_start + 1'b1;
					case(cnt_start)
						3'd0:	begin sda_out_r <= 1'b1; scl_out_r <= 1'b1; end	//将SCL和SDA拉高，保持4.7us以上
						3'd1:	begin sda_out_r <= 1'b1; scl_out_r <= 1'b1; end	//clk_400khz每个周期2.5us，需要两个周期
						3'd2:	begin sda_out_r <= 1'b0; end	//SDA拉低到SCL拉低，保持4.0us以上
						3'd3:	begin sda_out_r <= 1'b0; end	//clk_400khz每个周期2.5us，需要两个周期
						3'd4:	begin scl_out_r <= 1'b0; end	//SCL拉低，保持4.7us以上
						3'd5:	begin scl_out_r <= 1'b0; state <= MAIN; end	//clk_400khz每个周期2.5us，需要两个周期，返回MAIN
						default: state <= IDLE;	//如果程序失控，进入IDLE自复位状态
					endcase
				end
			WRITE:begin	//I2C通信时序中的写操作WRITE和相应判断操作ACK
					if(cnt <= 3'd6) begin	//共需要发送8bit的数据，这里控制循环的次数
						if(cnt_write >= 3'd3) begin cnt_write <= 1'b0; cnt <= cnt + 1'b1; end
						else begin cnt_write <= cnt_write + 1'b1; cnt <= cnt; end
					end else begin
						if(cnt_write >= 3'd7) begin cnt_write <= 1'b0; cnt <= 1'b0; end	//两个变量都恢复初值
						else begin cnt_write <= cnt_write + 1'b1; cnt <= cnt; end
					end
					case(cnt_write)
						//按照I2C的时序传输数据
						3'd0:	begin scl_out_r <= 1'b0; sda_out_r <= data_wr[7-cnt]; end	//SCL拉低，并控制SDA输出对应的位
						3'd1:	begin scl_out_r <= 1'b1; end	//SCL拉高，保持4.0us以上
						3'd2:	begin scl_out_r <= 1'b1; end	//clk_400khz每个周期2.5us，需要两个周期
						3'd3:	begin scl_out_r <= 1'b0; end	//SCL拉低，准备发送下1bit的数据
						//获取从设备的响应信号并判断
						3'd4:	begin sda_out_r <= 1'bz; end	//释放SDA线，准备接收从设备的响应信号
						3'd5:	begin scl_out_r <= 1'b1; end	//SCL拉高，保持4.0us以上
						3'd6:	begin if(i2c_sda) state <= IDLE; else state <= state; end	//获取从设备的响应信号并判断
						3'd7:	begin scl_out_r <= 1'b0; state <= MAIN; end	//SCL拉低，返回MAIN状态
						default: state <= IDLE;	//如果程序失控，进入IDLE自复位状态
					endcase
				end
			READ:begin	//I2C通信时序中的读操作READ和返回ACK的操作
					if(cnt <= 3'd6) begin	//共需要接收8bit的数据，这里控制循环的次数
						if(cnt_read >= 3'd3) begin cnt_read <= 1'b0; cnt <= cnt + 1'b1; end
						else begin cnt_read <= cnt_read + 1'b1; cnt <= cnt; end
					end else begin
						if(cnt_read >= 3'd7) begin cnt_read <= 1'b0; cnt <= 1'b0; end	//两个变量都恢复初值
						else begin cnt_read <= cnt_read + 1'b1; cnt <= cnt; end
					end
					case(cnt_read)
						//按照I2C的时序接收数据
						3'd0:	begin scl_out_r <= 1'b0; sda_out_r <= 1'bz; end	//SCL拉低，释放SDA线，准备接收从设备数据
						3'd1:	begin scl_out_r <= 1'b1; end	//SCL拉高，保持4.0us以上
						3'd2:	begin rtc_data_r[7-cnt] <= i2c_sda; end	//读取从设备返回的数据
						3'd3:	begin scl_out_r <= 1'b0; end	//SCL拉低，准备接收下1bit的数据
						//向从设备发送响应信号
						3'd4:	begin sda_out_r <= ack; end	//发送响应信号，将前面接收的数据锁存
						3'd5:	begin scl_out_r <= 1'b1; end	//SCL拉高，保持4.0us以上
						3'd6:	begin scl_out_r <= 1'b1; end	//SCL拉高，保持4.0us以上
						3'd7:	begin scl_out_r <= 1'b0; state <= MAIN; end	//SCL拉低，返回MAIN状态
						default: state <= IDLE;	//如果程序失控，进入IDLE自复位状态
					endcase
				end
			STOP:begin	//I2C通信时序中的结束STOP
					if(cnt_stop >= 3'd5) cnt_stop <= 1'b0;	//对STOP中的子状态执行控制cnt_stop
					else cnt_stop <= cnt_stop + 1'b1;
					case(cnt_stop)
						3'd0:	begin sda_out_r <= 1'b0; end	//SDA拉低，准备STOP
						3'd1:	begin sda_out_r <= 1'b0; end	//SDA拉低，准备STOP
						3'd2:	begin scl_out_r <= 1'b1; end	//SCL提前SDA拉高4.0us
						3'd3:	begin scl_out_r <= 1'b1; end	//SCL提前SDA拉高4.0us
						3'd4:	begin sda_out_r <= 1'b1; end	//SDA拉高
						3'd5:	begin sda_out_r <= 1'b1; state <= MAIN; end	//完成STOP操作，返回MAIN状态
						default: state <= IDLE;	//如果程序失控，进入IDLE自复位状态
					endcase
				end
			SHT_MODE1:begin	//单次写操作
					if(cnt_mode1 >= 4'd4) cnt_mode1 <= 1'b0;	//对START中的子状态执行控制cnt_start
					else cnt_mode1 <= cnt_mode1 + 1'b1;
					state_back <= SHT_MODE1;
					case(cnt_mode1)
						4'd0:	begin state <= SHT_START; end	//I2C通信时序中的START
						4'd1:	begin data_wr <= dev_addr<<1; state <= SHT_WRITE; end	//设备地址
						4'd2:	begin data_wr <= reg_addr; state <= SHT_WRITE; end	//寄存器地址
						4'd3:	begin state <= SHT_STOP; end	//I2C通信时序中的STOP
						4'd4:	begin state <= MAIN; end	//返回MAIN
						default: state <= IDLE;	//如果程序失控，进入IDLE自复位状态
					endcase
				end
			SHT_MODE2:begin	//两次读操作
					if(cnt_mode2 >= 4'd7) cnt_mode2 <= 4'd0;	//对START中的子状态执行控制cnt_start
					else cnt_mode2 <= cnt_mode2 + 1'b1;
					state_back <= SHT_MODE2;
					case(cnt_mode2)
						4'd0:	begin state <= SHT_START; end	//I2C通信时序中的START
						4'd1:	begin data_wr <= (dev_addr<<1)|8'h01; state <= SHT_WRITE; end	//设备地址
						4'd2:	begin ack <= ACK; state <= SHT_READ; end	//读寄存器数据
						4'd3:	begin dat_h <= data_r; end
						4'd4:	begin ack <= NACK; state <= SHT_READ; end	//读寄存器数据
						4'd5:	begin dat_l <= data_r; end
						4'd6:	begin state <= SHT_STOP; end	//I2C通信时序中的STOP
						4'd7:	begin state <= MAIN; end	//返回MAIN
						default: state <= IDLE;	//如果程序失控，进入IDLE自复位状态
					endcase
				end
			SHT_START:begin	//I2C通信时序中的起始START
					if(cnt_start >= 3'd5) cnt_start <= 1'b0;	//对START中的子状态执行控制cnt_start
					else cnt_start <= cnt_start + 1'b1;
					case(cnt_start)
						3'd0:	begin sda_out_r <= 1'b1; scl_out_r <= 1'b1; end	//将SCL和SDA拉高，保持4.7us以上
						3'd1:	begin sda_out_r <= 1'b1; scl_out_r <= 1'b1; end	//clk_400khz每个周期2.5us，需要两个周期
						3'd2:	begin sda_out_r <= 1'b0; end	//SDA拉低到SCL拉低，保持4.0us以上_out_r
						3'd3:	begin sda_out_r <= 1'b0; end	//clk_400khz每个周期2.5us，需要两个周期
						3'd4:	begin scl_out_r <= 1'b0; end	//SCL拉低，保持4.7us以上
						3'd5:	begin scl_out_r <= 1'b0; state <= state_back; end	//clk_400khz每个周期2.5us，需要两个周期，返回MAIN
						default: state <= IDLE;	//如果程序失控，进入IDLE自复位状态
					endcase
				end
			SHT_WRITE:begin	//I2C通信时序中的写操作WRITE和相应判断操作ACK
					if(cnt <= 3'd6) begin	//共需要发送8bit的数据，这里控制循环的次数
						if(cnt_write >= 3'd3) begin cnt_write <= 1'b0; cnt <= cnt + 1'b1; end
						else begin cnt_write <= cnt_write + 1'b1; cnt <= cnt; end
					end else begin
						if(cnt_write >= 3'd7) begin cnt_write <= 1'b0; cnt <= 1'b0; end	//两个变量都恢复初值
						else begin cnt_write <= cnt_write + 1'b1; cnt <= cnt; end
					end
					case(cnt_write)
						//按照I2C的时序传输数据
						3'd0:	begin scl_out_r <= 1'b0; sda_out_r <= data_wr[7-cnt]; end	//SCL拉低，并控制SDA输出对应的位
						3'd1:	begin scl_out_r <= 1'b1; end	//SCL拉高，保持4.0us以上
						3'd2:	begin scl_out_r <= 1'b1; end	//clk_400khz每个周期2.5us，需要两个周期
						3'd3:	begin scl_out_r <= 1'b0; end	//SCL拉低，准备发送下1bit的数据
						//获取从设备的响应信号并判断
						3'd4:	begin sda_out_r <= 1'bz; end	//释放SDA线，准备接收从设备的响应信号
						3'd5:	begin scl_out_r <= 1'b1; end	//SCL拉高，保持4.0us以上
						3'd6:	begin ack_flag <= i2c_sda; end	//获取从设备的响应信号并判断
						3'd7:	begin scl_out_r <= 1'b0; if(ack_flag)state <= state; else state <= state_back; end //SCL拉低，如果不应答循环写
						default: state <= IDLE;	//如果程序失控，进入IDLE自复位状态
					endcase
				end
			SHT_READ:begin	//I2C通信时序中的读操作READ和返回ACK的操作
					if(cnt <= 3'd6) begin	//共需要接收8bit的数据，这里控制循环的次数
						if(cnt_read >= 3'd3) begin cnt_read <= 1'b0; cnt <= cnt + 1'b1; end
						else begin cnt_read <= cnt_read + 1'b1; cnt <= cnt; end
					end else begin
						if(cnt_read >= 3'd7) begin cnt_read <= 1'b0; cnt <= 1'b0; end	//两个变量都恢复初值
						else begin cnt_read <= cnt_read + 1'b1; cnt <= cnt; end
					end
					case(cnt_read)
						//按照I2C的时序接收数据
						3'd0:	begin scl_out_r <= 1'b0; sda_out_r <= 1'bz; end	//SCL拉低，释放SDA线，准备接收从设备数据
						3'd1:	begin scl_out_r <= 1'b1; end	//SCL拉高，保持4.0us以上
						3'd2:	begin data_r[7-cnt] <= i2c_sda; end	//读取从设备返回的数据
						3'd3:	begin scl_out_r <= 1'b0; end	//SCL拉低，准备接收下1bit的数据
						//向从设备发送响应信号
						3'd4:	begin sda_out_r <= ack; end	//发送响应信号，将前面接收的数据锁存
						3'd5:	begin scl_out_r <= 1'b1; end	//SCL拉高，保持4.0us以上
						3'd6:	begin scl_out_r <= 1'b1; end	//SCL拉高，保持4.0us以上
						3'd7:	begin scl_out_r <= 1'b0; state <= state_back; end	//SCL拉低，返回MAIN状态
						default: state <= IDLE;	//如果程序失控，进入IDLE自复位状态
					endcase
				end
			SHT_STOP:begin	//I2C通信时序中的结束STOP
					if(cnt_stop >= 3'd5) cnt_stop <= 1'b0;	//对STOP中的子状态执行控制cnt_stop
					else cnt_stop <= cnt_stop + 1'b1;
					case(cnt_stop)
						3'd0:	begin sda_out_r <= 1'b0; end	//SDA拉低，准备STOP
						3'd1:	begin sda_out_r <= 1'b0; end	//SDA拉低，准备STOP
						3'd2:	begin scl_out_r <= 1'b1; end	//SCL提前SDA拉高4.0us
						3'd3:	begin scl_out_r <= 1'b1; end	//SCL提前SDA拉高4.0us
						3'd4:	begin sda_out_r <= 1'b1; end	//SDA拉高
						3'd5:	begin sda_out_r <= 1'b1; state <= state_back; end	//完成STOP操作，返回MAIN状态
						default: state <= IDLE;	//如果程序失控，进入IDLE自复位状态
					endcase
				end
			SHT_DELAY:begin	//12ms延时
					if(cnt_delay >= num_delay) begin
						cnt_delay <= 1'b0;
						state <= MAIN; 
					end else cnt_delay <= cnt_delay + 1'b1;
				end
			default:;
		endcase
	end
end

assign	i2c_scl = scl_out_r;	//对SCL端口赋值
assign	i2c_sda = sda_out_r;	//对SDA端口赋值

endmodule
