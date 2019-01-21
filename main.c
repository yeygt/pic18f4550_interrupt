#define MMCSD_PIN_SELECT  PIN_D2
#define MMCSD_PIN_SCK     PIN_D4
#define MMCSD_PIN_MOSI    PIN_D3
#define MMCSD_PIN_MISO    PIN_D5 

#include <18F4550.h>
#device PASS_STRINGS = IN_RAM

#fuses HSPLL PLL2 CPUDIV1 NOPBADEN NOLVP NOWDT NOPROTECT BROWNOUT PUT
#use delay(clock = 48MHz)

#use rs232(baud=9600, xmit=PIN_C5, rcv=PIN_C4)
#use pwm(CCP1,TIMER=2,FREQUENCY=187000,STREAM=PWM2) 
#use fast_io(D)
#use fast_io(E)

#include <mmcsd_m.c>
#include <fat_m.c>
#include <flex_lcd.c>

int32 io, i;
int8 v = 0;
int16 kesme_say = 0;
int32 file_boyutu = 0;
int32 zo;

FILE myfile;

#INT_EXT
void kesme ()
{
    
    output_toggle(PIN_A0);
    lcd_gotoxy(7, 2);
    kesme_say += 1;
    printf(lcd_putc, "(%Ld kesme)", kesme_say);
     
}

void main(void) {
  setup_adc_ports(NO_ANALOGS);
  
  ext_int_edge(L_TO_H); 
  clear_interrupt(INT_EXT);    
  enable_interrupts(INT_EXT); 
  enable_interrupts(GLOBAL); 
  
  delay_ms(500); 
  pwm_on(PWM2);
  
  delay_ms(500);
  lcd_init();                                    // Initialize LCD module
  lcd_putc('\f');                                // Clear LCD

  output_high(PIN_A2);
  delay_ms(500);
   
  i = fat_init();
  
   if(i == 0){
    output_high(PIN_A1);
    lcd_gotoxy(1, 1);
    printf(lcd_putc, "Fat (Ok)");
   
   if(fatopen("/aa.wav", "r", &myfile) == 0) {
    
      lcd_gotoxy(1, 2);
      printf(lcd_putc, "Muzik (Ok)");
      output_high(PIN_A0);
      //disp_fat_stats();
      //fatprintfinfo(&myfile);
      
      file_boyutu=file_boyut(&myfile);
      fatflush(&myfile);
      fatseek(&myfile, 1000, SEEK_SET);
      zo = file_boyutu/100;
     
    lcd_gotoxy(1, 1);
    printf(lcd_putc, "MUZIK CALIYOR...");
    lcd_gotoxy(1, 2);
    printf(lcd_putc, "                ");
    output_high(PIN_A0);
    
    for (io = 0; io <= file_boyutu; io++) { 

    set_pwm1_duty((int16)fatgetc(&myfile));
    
    if(io%zo==0) {
    v += 1;
    lcd_gotoxy(1, 2);
    printf(lcd_putc, "%% %d  ", v);
    }
    
    if (v >= 99) {
    io = 0;
    fatseek(&myfile, 1000, SEEK_SET);
    v=0;
    }

    
    }
 
 } else {
      lcd_gotoxy(1, 2);
      printf(lcd_putc, "Muzik (Hata)");
 }
 
 } else {
    lcd_gotoxy(1, 1);
    printf(lcd_putc, "Fat (Hata)");
   }
}


