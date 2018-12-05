
#include <avr/io.h>
#define __DELAY_BACKWARD_COMPATIBLE__ 
#include <util/delay.h>

#include "i2c_master.h"
#include "bme280.h"
#include "lcd.h"

//Defines for identifying the current reading which is displayed
#define TEMPERATURE_SELECTED 0
#define PRESSURE_SELECTED 1
#define HUMIDITY_SELECTED 2

//Locations of LED and button
#define IO_DIR DDRD
#define IO_OUTPUT PORTD
#define IO_INPUT PIND
#define LED_PIN PIND6
#define BUTTON_PIN PIND5

//Keeps track of the button status and currently selected reading
int button_status = 0;
int selected_reading = TEMPERATURE_SELECTED;

//Initialise button and LED
void io_init(void){
	IO_DIR |= (1<<LED_PIN);
	IO_DIR &= ~(1<<BUTTON_PIN);
	IO_OUTPUT &= ~(1<<LED_PIN);
}

//Check if the button is released
int check_button_release(void){
	int new_status = (IO_INPUT&(1<<BUTTON_PIN))>>BUTTON_PIN;
	if(new_status!=button_status){
		button_status = new_status;
		if(button_status==0){
			return 1;
		}
	}
	return 0;
}


//BME280 Functions - Required by the bme280 library
//Use the i2c_master library for reading and writing using I2C

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

//Initialise the bme280 device, which is passed as a pointer
//Note: Can't name the function bme280_init, since this is already defined
uint8_t sensor_init(struct bme280_dev* dev){
	
	int8_t result = BME280_OK;

	dev->dev_id = BME280_I2C_ADDR_SEC;
	dev->intf = BME280_I2C_INTF;
	dev->read = bme280_i2c_read;
	dev->write = bme280_i2c_write;
	dev->delay_ms = bme280_delay_ms;
	
	result = bme280_init(dev);
	
	if(result!=BME280_OK){
		return result;
	}

	// Recommended mode of operation: Indoor navigation
	dev->settings.osr_h = BME280_OVERSAMPLING_2X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_1X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	
	uint8_t settings_sel;
	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_FILTER_SEL;
	
	result = bme280_set_sensor_settings(settings_sel, dev);
	
	return result;
}


//Update the bme280 measurements and print the current selected reading to the LCD screen
uint8_t update_reading(struct bme280_dev* dev,struct bme280_data* component_data){
	
	uint8_t result = BME280_OK;
	
	result = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
	
	if(result!=BME280_OK){
		return result;
	}
	
	dev->delay_ms(40);
	
	result = bme280_get_sensor_data(BME280_ALL, component_data, dev);
	
	if(result!=BME280_OK){
		return result;
	}
	
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
	
	return result;
}


int main(void) {
	
	//Initialise IO (button and LED)
	
	io_init();
	
	//Initialise LCD
	
	lcd_init();
	lcd_scroll_right();
	lcd_set_left_to_right();
	lcd_disable_cursor();
	lcd_clear();
	lcd_on();
	
	//Initialise I2C
	
	i2c_init();
		
	//Initialise BME280
	
	struct bme280_dev dev;
	struct bme280_data component_data;
	uint8_t result = BME280_OK;
	
	result = sensor_init(&dev);
	
	//If BME280 initialisation was successful, start the main loop
	
	if(result==BME280_OK){
		//Setup complete, turn green LED on
		IO_OUTPUT |= (1<<LED_PIN);
		
		//Display an initial reading
		result = update_reading(&dev,&component_data);
		
		//Continue to check the button and change the reading on the LCD
		//when the button is released
		while(result==BME280_OK){
			if(check_button_release()){
				result = update_reading(&dev,&component_data);
			}
			_delay_ms(10);
		}
	}
	
	//If the code reaches here, there has been an error
	lcd_clear();
	lcd_printf("Error: %i",result);
	return 1;
}