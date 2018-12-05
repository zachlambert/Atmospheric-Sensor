
#include <avr/io.h>
#define __DELAY_BACKWARD_COMPATIBLE__ 
#include <util/delay.h>
#include <util/twi.h>
#include "i2c_master.h"
#include "bme280.h"
#include "lcd.h"

#define TEMPERATURE_SELECTED 0
#define PRESSURE_SELECTED 1
#define HUMIDITY_SELECTED 2

#define IO_DIR DDRD
#define IO_OUTPUT PORTD
#define IO_INPUT PIND
#define LED_PIN PIND6
#define SWITCH_PIN PIND5

int switch_status = 0;
int selected_reading = TEMPERATURE_SELECTED;

void io_init(void){
	IO_DIR |= (1<<LED_PIN);
	IO_DIR &= ~(1<<SWITCH_PIN);
	IO_OUTPUT &= ~(1<<LED_PIN);
}

int check_switch_release(void){
	int new_status = (IO_INPUT&(1<<SWITCH_PIN))>>SWITCH_PIN;
	if(new_status!=switch_status){
		switch_status = new_status;
		if(switch_status==0){
			return 1;
		}
	}
	return 0;
}

void bme280_delay_ms(uint32_t period){
  _delay_ms(period);
}

int8_t bme280_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len){
    int8_t rslt = 0;

	if(i2c_start(dev_id<<1 | I2C_WRITE)) return 1;
    i2c_write(reg_addr);
    i2c_stop();
	
    if (i2c_start(dev_id<<1 | I2C_READ)) return 1;
    int i;
    for(i=0;i<len-1;i++){
      reg_data[i] = i2c_read_ack();
    }
	reg_data[len-1] = i2c_read_nack();
    i2c_stop();
	
    return rslt;
}

int8_t bme280_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len){
    int8_t rslt = 0;
	
    if(i2c_start(dev_id<<1 | I2C_WRITE)) return 1;
    i2c_write(reg_addr);
    
    int i;
    for(i=0;i<len;i++){
      i2c_write(reg_data[0]);
    }
    i2c_stop();

    return rslt;
}

void update_reading(struct bme280_dev* dev,struct bme280_data* component_data){
	
	uint8_t result = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
	
	dev->delay_ms(40);
	
	result = bme280_get_sensor_data(BME280_ALL, component_data, dev);
	
	lcd_clear();
	
	switch(selected_reading){
		case TEMPERATURE_SELECTED:
			lcd_printf("Temperature");
			lcd_set_cursor(0,1);
			lcd_printf("%ld",component_data->temperature/100);
			lcd_printf(".%ld",component_data->temperature%100);
			lcd_printf(" (Celsius)");
			break;
		case PRESSURE_SELECTED:
			lcd_printf("Pressure");
			lcd_set_cursor(0,1);
			lcd_printf("%ld",component_data->pressure/100);
			lcd_printf(" (Pa)");
			break;
		case HUMIDITY_SELECTED:
			lcd_printf("Humidity");
			lcd_set_cursor(0,1);
			lcd_printf("%ld",(component_data->humidity/1024));
			lcd_puts("%");
			break;
	}
	
	selected_reading += 1;
	selected_reading %= 3;
}

int main(void) {
	
	io_init();
	
	lcd_init();
	lcd_scroll_right();
	lcd_set_left_to_right();
	lcd_disable_cursor();
	
	lcd_clear();
	lcd_on();
	
	i2c_init();
		
	struct bme280_dev dev;
	int8_t result = BME280_OK;

	dev.dev_id = BME280_I2C_ADDR_SEC;
	dev.intf = BME280_I2C_INTF;
	dev.read = bme280_i2c_read;
	dev.write = bme280_i2c_write;
	dev.delay_ms = bme280_delay_ms;
	
	result = bme280_init(&dev);
	
	if(result!=BME280_OK){
		lcd_printf("Error: %i",result);
		return;
	}
	
	struct bme280_data component_data;
	
	// Recommended mode of operation: Indoor navigation
	dev.settings.osr_h = BME280_OVERSAMPLING_2X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_1X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	
	uint8_t settings_sel;	
	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_FILTER_SEL;
	
	result = bme280_set_sensor_settings(settings_sel, &dev);
	
	int selected_reading = 0;
	
	if(result==0){
		//Setup complete, turn green LED on
		IO_OUTPUT |= (1<<LED_PIN);
	}
	
	update_reading(&dev,&component_data);
	while(result==0){
		if(check_switch_release()){
			update_reading(&dev,&component_data);
		}else{
			//lcd_clear();
			//lcd_printf("%x",IO_INPUT);
			//lcd_set_cursor(0,1);
			//lcd_printf("%d",(IO_INPUT&(1<<SWITCH_PIN))>>SWITCH_PIN);
		}
		_delay_ms(10);
	}	
	
	if(result!=0){
		lcd_clear();
		lcd_printf("Error: %i",result);
	}
}