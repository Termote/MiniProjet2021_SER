#include <camera/po8030.h>
#include "camera/dcmi_camera.h"
#include <process_image.h>


static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle of image

/**************************** SEMAPHORES *************************************/
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/************************ INTERNAL FUNCTIONS *********************************/

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = 0;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)){

			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean){
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin){
		    stop = 0;

		    while(stop == 0 && i < IMAGE_BUFFER_SIZE){

		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean){
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end){
		        line_not_found = 1;
		    }
		}
		//if no begin was found
		else{ 				
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = last_width;
	}
	else {
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		line_not_found = 0;
		return PXTOCM/MAX_DISTANCE;
	}
	else {
		line_not_found =0;
		return width;
	}
}
/********************** END INTERNAL FUNCTIONS *******************************/
/****************************** THREADS **************************************/

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){

        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint16_t lineWidth = 0;
	uint8_t blue_values = 0;
	uint8_t green_values = 0;

    while(1){

    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		// extract blue and green

		for(uint16_t i = 0; i < (IMAGE_BUFFER_SIZE*2) ; i++){

			green_values = (img_buff_ptr[i] & 0x07) << 3;    // perform mask to only extract needed pixels
			blue_values = (img_buff_ptr[i] & 0x00);

			green_values = (green_values | ((img_buff_ptr[++i] & 0xE0) >> 5));
			blue_values = (img_buff_ptr[i] & 0x1F );

			image[i/2] = green_values + blue_values;
		}

		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image);

		//converts the width into a distance between the robot and the camera
		if(lineWidth){
			distance_cm = PXTOCM/lineWidth;
		}
		lineWidth = 0;
    }
}

/**************************** END THREADS *************************************/
/************************* PUBLIC FUNCTIONS **********************************/

float get_distance_cm(void) {

	return distance_cm;
}

uint16_t get_line_position(void) {

	return line_position;
}

void process_image_start(void) {

	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

/*********************** END PUBLIC FUNCTIONS ********************************/