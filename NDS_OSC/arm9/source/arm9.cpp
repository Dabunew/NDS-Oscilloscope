#include <nds.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <osc_scale.h>
#include <osc_logo.h>
#include <osc_sprite.h>
#include <maxmod9.h>
#include "mmsolution.h"		// solution definitions
#include "mmsolution_bin.h"	// solution binary reference 
#include "ifft.h"

#define hi(v)	(*(((int8*)(&v)+1)))
#define lo(v)	(*((int8*)(&v)))
#define Green	0x83E0
#define Blue	0xFC00
#define Red 	0x801F
#define White	0xFFFF
#define Black	0x8000
#define Trans	0x0000
#define MIC		true
#define SLOT2	false
#define plot_max 0x80

u32		sample_rate =32768;	//the record sample rate
u16* 	data_buffer =0;	//buffer which is written to by the arm7
u16*	dold_buffer =0;
u32		data_bytesize =1024;	//the mic buffer sent to the arm7 is a double buffer every time it is half full the arm7 signals us so we can read the data. 
bool	data_source = MIC;
u16*	plot_buffer	=0;
u8		plot_scale =1;
u8		plot_offset_x =50;
s8		plot_offset_y =0;
u8		cursor_x=0;
s16		cursor_y=0;
bool	plot_update;
bool 	running=false;
u8		trigger_type=0;
u8		trigger_channel=0;
s8		trigger_level=25;
bool	trigger_edge=true;
u8		trigger_state=0;
u16		trigger_pos=0;
u8		user_mode=0;
const char* str_mode[4]={"Oscilloscope", "FFT Analyzer", "Logic Scope ", "SigGenerator"};
const char* str_trigger[3]={" FREE ", "SINGLE", " CONT."};
const char* str_scale[8]={"/2", "x1", "x2", "x4", "x8", "x16", "x32", "x64"};

typedef struct{ int x; int y; u16* gfx_mem;} OSC_sprite;
OSC_sprite sprite_ox={53,0x80}, sprite_oy={0,61}, sprite_cx={0,0x80}, sprite_cy={0,61}, sprite_cc={4,62}, sprite_tg={0,62}, sprite_heart;

void plotlogic(u16* buffer){
	u16	ipixel, tmp_data; 
	u8*	pt_lo=(u8*)(&ipixel);
	u8*	pt_hi=(u8*)(&ipixel)+1;
	u8 idata;
	s8 ich;
	cursor_y=buffer[cursor_x];
	for(idata=0; idata<250; idata++){
		(*pt_lo)=idata+6;
		tmp_data=buffer[idata];
		(*pt_hi)=0;
		for(ich=15; ich>=0; ich--){
			for(; (*pt_hi)<(120-ich*7-(tmp_data&BIT(ich)?2:0)); (*pt_hi)++) plot_buffer[ipixel]=Black;
			plot_buffer[ipixel]=((ich==trigger_channel)?Red:Green);
			(*pt_hi)++;
		}
		for(; (*pt_hi)<=plot_max;	(*pt_hi)++)	plot_buffer[ipixel]=Trans;
	}
}

void fillcurve(u16* buffer){
	u16	ipixel, color1, color2; 
	u8*	pt_lo=(u8*)(&ipixel);
	u8*	pt_hi=(u8*)(&ipixel)+1;
	u8	imax, imid, imin, idata;
	s8	offset_y=plot_offset_y+(user_mode?0x0:0x40);
	s16 data1, data2;
	cursor_y=trigger_channel?lo(buffer[cursor_x]):hi(buffer[cursor_x]);
	for(idata=0; idata<250; idata++){
		(*pt_lo)=idata+6;
		data1 =hi(buffer[idata]);
		if(data_source==SLOT2) data2 =lo(buffer[idata]); else data2=0;
		plot_scale?(data1<<=plot_scale-1):(data1>>=1);	data1 +=offset_y;
		plot_scale?(data2<<=plot_scale-1):(data2>>=1);	data2 +=offset_y;
		if(data1<data2){
			color1=Green;	color2=Red;
			if(data1<offset_y){		imin=data1;
				if(data2<offset_y){	imid=data2;	imax=offset_y;}	else{imid=offset_y;	imax=data2;}
			}else{	imin=offset_y;	imid=data1;	imax=data2;}
		}else{
			color1=Red;	color2=Green;
			if(data2<offset_y){		imin=data2;
				if(data1<offset_y){	imid=data1;	imax=offset_y;}	else{imid=offset_y;	imax=data1;}
			}else{	imin=offset_y;	imid=data2;	imax=data1;}
		}
		if(imin>plot_max) imin=plot_max;	if(imid>plot_max) imid=plot_max;	if(imax>plot_max) imax=plot_max;
		imin=plot_max-imin; imid=plot_max-imid; imax=plot_max-imax;
		(*pt_hi)=0;
		for(; (*pt_hi)<imax;	(*pt_hi)++)	plot_buffer[ipixel]=Trans;
		for(; (*pt_hi)<=imid;	(*pt_hi)++)	plot_buffer[ipixel]=color2;
		for(; (*pt_hi)<=imin;	(*pt_hi)++)	plot_buffer[ipixel]=color1;
		for(; (*pt_hi)<=plot_max;	(*pt_hi)++)	plot_buffer[ipixel]=Trans;
	}
}

void daqstop(void);

void micHandler(void* data, int length){	//mic stream handler
	if(!running) return;
	if(length<500) return;
	if(plot_update) return;
	DC_InvalidateRange(data, length);
	u16* tmp_buffer=(u16*)data;
	s8 data_tmp, trigger;
	u16 id=0;
	if(trigger_type){
		trigger=(user_mode==2)?trigger_edge:trigger_level;
		switch(trigger_state){
			case 0:	for(id=0;id<256;id++){
						if(user_mode==2) data_tmp=((tmp_buffer[id]&BIT(trigger_channel))?1:0);
						else data_tmp=trigger_channel?lo(tmp_buffer[id]):hi(tmp_buffer[id]);
						if(trigger_edge?(data_tmp<trigger):(data_tmp>trigger)){
							trigger_state++;
							break;
						}
					}//waiting for initialization
			case 1:	for(;id<256;id++){
						if(user_mode==2) data_tmp=((tmp_buffer[id]&BIT(trigger_channel))?1:0);
						else data_tmp=trigger_channel?lo(tmp_buffer[id]):hi(tmp_buffer[id]);
						if(trigger_edge?(data_tmp>=trigger):(data_tmp<=trigger)){
							if(id<=plot_offset_x){
								trigger_pos=(plot_offset_x-id)<<1;
								dmaCopy(((u8*)dold_buffer)+length-trigger_pos, (u8*)dold_buffer, trigger_pos);
								dmaCopy(data, ((u8*)dold_buffer)+trigger_pos, length-trigger_pos);
								plot_update=true;
								trigger_state=0;
								if(trigger_type==1) daqstop();
							}else{
								trigger_pos=(id-plot_offset_x)<<1;
								dmaCopy(((u8*)data)+trigger_pos, (u8*)dold_buffer, length-trigger_pos);
								trigger_state++; 
							}	
							break;
						}
					}break;//initialized
			case 2:	dmaCopy(data, ((u8*)dold_buffer)+length-trigger_pos, trigger_pos);
					plot_update=true;
					trigger_state=0;
					if(trigger_type==1)	daqstop();
					break;//triggered
			default:	;
		}
		if((!plot_update)&(trigger_state<2))	dmaCopy(data, (u8*)dold_buffer, length);
	}else{
		dmaCopy(data, (u8*)dold_buffer, length);
		plot_update=true;
	}
}

void screenInit(void){
	lcdSwap();
	videoSetMode(MODE_5_2D); //set the mode for 2 text layers and two extended background layers
	videoSetModeSub(MODE_5_2D); //Text Mode
	vramSetPrimaryBanks(VRAM_A_MAIN_BG_0x06000000, VRAM_B_MAIN_BG_0x06020000, VRAM_C_SUB_BG, VRAM_D_MAIN_BG_0x06040000); 

	plot_buffer =(u16*)bgGetGfxPtr(bgInitSub(3, BgType_Bmp16, BgSize_B16_256x256, 1,0));	//second buffer 16x16KB offset
	decompress(osc_logoBitmap, plot_buffer,  LZ77Vram);
    consoleInit(NULL, 1,BgType_Text4bpp, BgSize_T_256x256, 0, 7, false, true);	//31*2KB offset
 	iprintf("\x1b[32;1m  <>:\x1b[39mPlot Offset X\n");
	iprintf("\x1b[32;1m  ^V:\x1b[39mPlot Offset Y\n\n");
	iprintf("\x1b[32;1mL+<>:\x1b[39mSampling Rate\n");
	iprintf("\x1b[32;1mL+^V:\x1b[39mY Scale\n\n");
	iprintf("\x1b[32;1mR+<>:\x1b[39mTrigger Channel\n");
	iprintf("\x1b[32;1mR+^V:\x1b[39mTrigger Level\n\n");
	iprintf("\x1b[32;1m A  :\x1b[39mRun/Stop\n");
	iprintf("\x1b[32;1m B  :\x1b[39mMicrophone/Slot2\n\n");
	iprintf("\x1b[32;1m X  :\x1b[39mNo Trigger/Single/Continous");
	iprintf("\x1b[32;1m Y  :\x1b[39mRising/Falling Edge\n\n");
	iprintf("\x1b[32;1mStart :\x1b[39mBacklight\n");	
	iprintf("\x1b[32;1mSelect:\x1b[39mScope/FFT/Logic Analyzer\n\n");
	iprintf("\x1b[32;1m  PEN :\x1b[39mMove Cursor\n");
	iprintf("\x1b[32;1mL+PEN :\x1b[39mZoom Plot\n");
	
	consoleInit(NULL, 1,BgType_Text4bpp, BgSize_T_256x256, 0, 7, true, true);		//7*16KB offset, 1~6 16KB is allocated for bg2
//	consoleSetWindow (NULL, 0, 16, 32, 8);
//	consoleSelect(&topScreen);
	plot_buffer =(u16*)bgGetGfxPtr(bgInit(3, BgType_Bmp16, BgSize_B16_256x256, 16,0));	//second buffer 16x16KB offset
	decompress(osc_scaleBitmap, plot_buffer,  LZ77Vram);
	
	vramSetBankE(VRAM_E_MAIN_SPRITE);
	vramSetBankI(VRAM_I_SUB_SPRITE);
	oamInit(&oamMain, SpriteMapping_1D_128, false);
	oamInit(&oamSub, SpriteMapping_1D_128, false);
	
	u8 sprite_size=8*8;
	sprite_ox.gfx_mem=oamAllocateGfx(&oamMain, SpriteSize_8x8, SpriteColorFormat_256Color);
	dmaCopy(osc_spriteTiles, sprite_ox.gfx_mem, sprite_size);
	sprite_oy.gfx_mem=sprite_ox.gfx_mem;
	sprite_cx.gfx_mem=oamAllocateGfx(&oamMain, SpriteSize_8x8, SpriteColorFormat_256Color);
	dmaCopy(osc_spriteTiles+sprite_size*1/4, sprite_cx.gfx_mem, sprite_size);
	sprite_cy.gfx_mem=sprite_cx.gfx_mem;
	sprite_cc.gfx_mem=oamAllocateGfx(&oamMain, SpriteSize_8x8, SpriteColorFormat_256Color);
	dmaCopy(osc_spriteTiles+sprite_size*2/4, sprite_cc.gfx_mem, sprite_size);
	sprite_tg.gfx_mem=oamAllocateGfx(&oamMain, SpriteSize_8x8, SpriteColorFormat_256Color);
	dmaCopy(osc_spriteTiles+sprite_size*3/4, sprite_tg.gfx_mem, sprite_size);
	sprite_heart.gfx_mem=oamAllocateGfx(&oamSub, SpriteSize_8x8, SpriteColorFormat_256Color);
	dmaCopy(osc_spriteTiles+sprite_size*4/4, sprite_heart.gfx_mem, sprite_size);
	
	dmaCopy(osc_spritePal, SPRITE_PALETTE, 512);
	dmaCopy(osc_spritePal, SPRITE_PALETTE_SUB, 512);
}

void oamSetXY(int id, const void* gfx_mem, int x, int y){oamSet(&oamMain, id, x, y, 0, 0, SpriteSize_8x8, SpriteColorFormat_256Color, gfx_mem, -1, false, false, false, false, false);} 
void plotoam(){
	if(user_mode==2) sprite_cc.y=117-trigger_channel*7-(cursor_y&BIT(trigger_channel)?2:0);
	else sprite_cc.y=(user_mode?125:61)-(plot_scale?(cursor_y<<(plot_scale-1)):(cursor_y>>1))-plot_offset_y;
	oamSetXY(5, sprite_ox.gfx_mem, plot_offset_x+3, sprite_ox.y);
	oamSetXY(4, sprite_oy.gfx_mem, sprite_oy.x, 61-plot_offset_y);
	oamSetXY(3, sprite_cx.gfx_mem, cursor_x+3, sprite_cx.y);
	oamSetXY(2, sprite_cy.gfx_mem, sprite_cy.x, sprite_cc.y);
	oamSetXY(1, sprite_cc.gfx_mem, cursor_x+4, sprite_cc.y);
	oamSetXY(0, sprite_tg.gfx_mem, sprite_tg.x, 61-(plot_scale?(trigger_level<<(plot_scale-1)):(trigger_level>>1))-plot_offset_y);
	oamSet(&oamSub, 0, rand()%241, rand()%178, 0, 0, SpriteSize_8x8, SpriteColorFormat_256Color, sprite_ox.gfx_mem, 0, true, false, false, false, false);
	oamRotateScale(&oamSub, 0, 0, rand()%128+128, rand()%128+128);
	oamUpdate(&oamMain);
	oamUpdate(&oamSub);
}

void slot2on(){soundMicRecord(data_buffer, data_bytesize, MicFormat_12Bit, sample_rate, micHandler);}
void slot2off(){soundMicOff();}
void daqstart(void){
	if(data_source==MIC) soundMicRecord(data_buffer, data_bytesize, MicFormat_12Bit, sample_rate, micHandler); else slot2on();
	trigger_state=0;
	running=true;
}
void daqstop(void){
	running=false;
	if(data_source==MIC) soundMicOff(); else slot2off();
}

mm_word myEventHandler( mm_word msg, mm_word param ) {	//sound events
	switch( msg ) {
		case MMCB_SONGMESSAGE:	break;// process song messages
								//if (param == 1) sprite_heart.x += 20;	// if song event 1 is triggered, set sprite's y velocity to make it jump
		case MMCB_SONGFINISHED:	break;// process song finish message (only triggered in songs played with MM_PLAY_ONCE)
    }return 0;
}

int main(void) {
	data_buffer =(u16*)malloc(data_bytesize);
	dold_buffer =(u16*)malloc(1024);
	screenInit();
	int bg = bgInit(2, BgType_Bmp16, BgSize_B16_256x256, 1,0);	//1*16kB offset, fist 16kb is allocated for console text.
	plot_buffer =(u16*)bgGetGfxPtr(bg);
	bgInit(2, BgType_Bmp16, BgSize_B16_256x256, 8,0);

	mmInitDefaultMem((mm_addr)mmsolution_bin);	// initialise maxmod using default settings, and enable interface for a soundbank that is loaded into memory
	mmSetEventHandler(myEventHandler);	// setup maxmod to use the song event handler
	mmLoad(MOD_REMEMBER);	// load song. values for this function are in the solution header
	mmStart(MOD_REMEMBER, MM_PLAY_LOOP);	// start the music playing
	mmSetModuleVolume(1024);
	
	u32	counter=0;
	touchPosition touch;
	keysSetRepeat(50,5);
	while(true){
		swiWaitForVBlank();
		if(plot_update){
			switch(user_mode){
				 case 0:	fillcurve(dold_buffer);	break;
				 case 1:	ifft(dold_buffer, dold_buffer+256); fillcurve(dold_buffer+256);	break;
				 case 2:	plotlogic(dold_buffer);	break;
				 default:	;
			}
			swiWaitForVBlank();
			plot_buffer =(u16*)bgGetGfxPtr(bg);	//swap the back buffer to the current buffer 8*16KB offset
			if(bgGetMapBase(bg)==1)	bgSetMapBase(bg, 8);
			else	bgSetMapBase(bg, 1);
			plot_update =false;
		}
		scanKeys();
		int key =keysDownRepeat();
		if(!key) key=keysDown();
		if(key) key|=keysHeld();
		switch(key){
			case KEY_A:		if(!running) daqstart(); else daqstop();							break;
			case KEY_B:		if(running) {daqstop(); running=false;} data_source=!data_source;	break;
			case KEY_X:		trigger_type++; trigger_type%=3; trigger_state=0;					break;
			case KEY_Y:		trigger_edge=!trigger_edge;	trigger_state=0;						break;
			case KEY_SELECT:user_mode++; user_mode%=3;											break;
			case KEY_UP:	if(plot_offset_y<63) plot_offset_y++; 
							if(!running) plot_update=true;										break;
			case KEY_DOWN:	if(plot_offset_y>-64) plot_offset_y--;
							if(!running) plot_update=true;										break;
			case KEY_LEFT:	if(plot_offset_x>0) plot_offset_x--; 								break;
			case KEY_RIGHT:	if(plot_offset_y<255) plot_offset_x++;								break;
			case KEY_L|KEY_UP:		if(plot_scale<7) plot_scale++;
									if(!running) plot_update=true;								break;
			case KEY_L|KEY_DOWN:	if(plot_scale>0) plot_scale--;	
									if(!running) plot_update=true;								break;
			case KEY_L|KEY_LEFT:	if(sample_rate>1024) sample_rate/=2;						break;
			case KEY_L|KEY_RIGHT:	if(sample_rate<32768) sample_rate*=2;						break;
			case KEY_R|KEY_UP:		if(trigger_level<127) trigger_level++;						break;
			case KEY_R|KEY_DOWN:	if(trigger_level>-128) trigger_level--;					break;
			case KEY_R|KEY_LEFT:	if(trigger_channel>0) trigger_channel--;					break;
			case KEY_R|KEY_RIGHT:	if(trigger_channel<15) trigger_channel++;					break;
			case KEY_TOUCH:			touchRead(&touch);	cursor_x=touch.px-6;
									if(user_mode==2) cursor_y=dold_buffer[cursor_x];
									else cursor_y=trigger_channel?lo(dold_buffer[cursor_x+((user_mode==1)?256:0)]):hi(dold_buffer[cursor_x+((user_mode==1)?256:0)]);
									break;
			case KEY_L|KEY_TOUCH:	touchRead(&touch); 
									bgSetScale(bg, (192-touch.py)*256/192, 256); bgSetScroll(bg, touch.px, 0);
									bgUpdate();	break;
			case KEY_L:				bgSetScale(bg, 256, 256); bgSetScroll(bg, 0, 0);
									bgUpdate();	break;
			default:		;
		}
		if(key) counter=0;
		if(!(counter&0x01F)){
			if(user_mode==2) iprintf("\x1b[0;1H\x1b[33;1mY:%04X", dold_buffer[cursor_x]);
			else iprintf("\x1b[0;1H\x1b[33;1mY:%-4d", cursor_y);
			iprintf("\x1b[15;27H\x1b[33;1mX:%-3d", cursor_x);
			// print at using ansi escape sequence \x1b[line;columnH 
			iprintf("\x1b[17;0H\x1b[33;1m%s", str_mode[user_mode]);
			iprintf("\x1b[17;13H\x1b[39mDataSource:\x1b[32;1m%s", (data_source==MIC)?"MIC. ":"SLOT2"); 
			iprintf("\x1b[17;29H\x1b[31;1m%s", running?"RUN":"STP");
			
			iprintf("\x1b[19;0H\x1b[39mFREQ:\x1b[32;1m%5d\x1b[34;1mHz", sample_rate);
			iprintf("\x1b[19;12H\x1b[39mSCALE:\x1b[32;1m%3s", str_scale[plot_scale]);
			iprintf("\x1b[19;22H\x1b[39mOFFSET:\x1b[32;1m%3d\x1b[20;29H%3d", plot_offset_x, plot_offset_y);

			iprintf("\x1b[21;0H\x1b[39mTRIG:\x1b[32;1m%s", str_trigger[trigger_type]);
			iprintf("\x1b[21;12H\x1b[39mLEVEL:\x1b[32;1m%3d", trigger_level);
			iprintf("\x1b[21;22H\x1b[39mEDGE:\x1b[32;1m%s", trigger_edge?"_/":"\\_");
			iprintf("\x1b[21;29H\x1b[39m@\x1b[33;1m%2d", trigger_channel);

			time_t ctime =time(NULL);
			tm *ptime;
			ptime =(tm*)gmtime((const time_t *)&ctime);
			if(running) iprintf("\x1b[23;0H\x1b[32;1m%d-%02d-%02d %02d:%02d:%02d", ptime->tm_year+1900,ptime->tm_mon+1, ptime->tm_mday, ptime->tm_hour, ptime->tm_min, ptime->tm_sec);
			s32 temperature=0;
			fifoSendValue32(FIFO_USER_01,0);
			if(fifoCheckValue32(FIFO_USER_01))	temperature =fifoGetValue32(FIFO_USER_01);	//20.12 format
			iprintf("\x1b[23;23H\x1b[39mT:\x1b[32;1m%3d.%1d\x1b[34;1m'C", temperature>>12, ((temperature&0x3FF)/100)%10);
			plotoam();
			if(!running) plot_update=true;
		}
		counter++;
	}
	return 0;
}
