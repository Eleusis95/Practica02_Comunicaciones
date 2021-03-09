/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "lwip/opt.h"

#if LWIP_IPV4 && LWIP_RAW && LWIP_NETCONN && LWIP_DHCP && LWIP_DNS

#include "lwip/api.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dhcp.h"
#include "lwip/netdb.h"
#include "lwip/netifapi.h"
#include "lwip/prot/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"

#include "ctype.h"

#include "board.h"

#include "fsl_device_registers.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "event_groups.h"
#include "stdlib.h"
#include "stdio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* MAC address configuration. */
#define configMAC_ADDR                     \
    {                                      \
        0x02, 0x12, 0x13, 0x10, 0x15, 0x11 \
    }

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS

/* System clock name. */
#define EXAMPLE_CLOCK_NAME kCLOCK_CoreSysClk

/* GPIO pin configuration. */
#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN
#define BOARD_SW_GPIO BOARD_SW3_GPIO
#define BOARD_SW_GPIO_PIN BOARD_SW3_GPIO_PIN
#define BOARD_SW_PORT BOARD_SW3_PORT
#define BOARD_SW_IRQ BOARD_SW3_IRQ
#define BOARD_SW_IRQ_HANDLER BOARD_SW3_IRQ_HANDLER


/*! @brief MQTT client ID. */
#define EXAMPLE_MQTT_CLIENT_ID "k64"

//#define LOCAL_BROKER

#ifdef LOCAL_BROKER
/*! @brief MQTT server host name or IP address. */
#define EXAMPLE_MQTT_SERVER_HOST "io.adafruit.com"
//192.168.15.5
#define EXAMPLE_MQTT_USER NULL
#define EXAMPLE_MQTT_PSWD NULL

/*! @brief MQTT server port number. */
#define EXAMPLE_MQTT_SERVER_PORT 1883
#else
/*! @brief MQTT server host name or IP address. */
#define EXAMPLE_MQTT_SERVER_HOST "io.adafruit.com"

#define EXAMPLE_MQTT_USER "Eleusis_95"
#define EXAMPLE_MQTT_PSWD "aio_rVSA19wQT3aL20dUXC8RLjTLwwo5"

/*! @brief MQTT server port number. */
#define EXAMPLE_MQTT_SERVER_PORT 1883
#endif
//13862
/*! @brief Stack size of the temporary lwIP initialization thread. */
#define APP_THREAD_STACKSIZE 3024

/*! @brief Priority of the temporary lwIP initialization thread. */
#define APP_THREAD_PRIO DEFAULT_THREAD_PRIO

#define MQTT_CONNECTED_EVT		( 1 << 0 )
#define MQTT_SENSOR_EVT			( 1 << 1 )
#define MQTT_SPRINKLERS_EVT		( 1 << 2 )
#define MQTT_DISCONNECTED_EVT	( 1 << 3 )
/*My publishers and subscribers*/
#define MQTT_JOY_X_EVT			( 1 << 4 ) /*Potentiometer Publisher:  value from Joystick axi X)_OK*/
#define MQTT_SW_EVT				( 1 << 5 )/*SW subscriber: a switch, from k64 is going to turn on/off a led on adrafruit dashboard_OK*/
#define MQTT_SET_SPEED_EVT		( 1 << 6 )/*Set speed subscriber:  a pwm signal is going to be set in base to the value from this topic()*/
#define MQTT_HALL_SENSOR_EVT	( 1 << 7 )/*Hall sensor publisher: it is going to publisg at adafruit dashboard if a magnetic field is detected(publisher)*/
#define MQTT_MOTOR_READ_SPEED_EVT ( 1 << 8)/*Motor read: to read the motor speed in rpm(publisher)*/


/*Macros to control leds*/
#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

#define MOTOR_TOGGLE() LED_BLUE_TOGGLE()
#define TOGGLE_MOTOR() LED_GREEN_TOGGLE()

#define MAX_CMD_LENGTH 2
#define MAX_USERS_LENGTH 50
QueueHandle_t servo_queue = NULL; /*queue de servo*/
QueueHandle_t speed_por_queue = NULL; /*motor speed queue*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void connect_to_mqtt(void *ctx);
static int32_t get_simulated_sensor(int32_t current_value, int32_t max_step,
		                     int32_t min_value, int32_t max_value,
							 bool increase);
static void sensor_timer_callback(TimerHandle_t pxTimer);
static void publish_humidity(void *ctx);


extern void Init_JoyStick(void);/*init for ADC for joystick*/
static void publish_Joystick(void *ctx);/*Joystick publisher*/
static void joystick_stick_callback(TimerHandle_t pxTimer);/*Joystick callback*/
extern uint32_t Joystick_position(void);/*setting the pot value*/

static void publish_hall_sensor(void *ctx);/*Hall sensor publisher*/
static void hall_sesnsor_callback(TimerHandle_t pxTimer);/*hall sensor callback*/
//extern void hall_init(void);/*Initialitation for hall sensor setting*/

/*Functions for motors*/
static void toggle_motor(void); /*Turning on/off motor*/
static void change_motor_turn(void);/*turnign left/right motor*/


//extern void Init_Hall_sensor(void);
//extern void servo_task(void);

//extern uint32_t motor_active(void);

/*velocidad del motor*/
extern void init_motor_calculate_speed(void); /*setting for calculate motor speed*/
int motor_speed(void);/*gives back the quantity of encoder activations*/
static void hall_motor_calculate_speed_callback(TimerHandle_t pxTimer);/*motor speed callback*/
static double calculate_speed(int vueltas);/*calculate the motor speed in base to the activations*/
/*speed motor*/
void setting_speed(void);/*catch the value from a queue*/
void Init_set_motor_speed(void);/*setting for set the motor speed in base to a PWM signal*/
void turn_on_pwm(void);/*turn on the motor*/
void turn_off_pwm(void);/*turn of the motor*/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief MQTT client data. */
static mqtt_client_t *mqtt_client;

/*! @brief MQTT client information. */
static const struct mqtt_connect_client_info_t mqtt_client_info = {
    .client_id   = EXAMPLE_MQTT_CLIENT_ID,
    .client_user = EXAMPLE_MQTT_USER,
    .client_pass = EXAMPLE_MQTT_PSWD,
    .keep_alive  = 100,
    .will_topic  = NULL,
    .will_msg    = NULL,
    .will_qos    = 0,
    .will_retain = 0,
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    .tls_config = NULL,
#endif
};

/*! @brief MQTT broker IP address. */
static ip_addr_t mqtt_addr;

/*! @brief Indicates connection to MQTT broker. */
static volatile bool connected = false;

// Declare a variable to hold the created event group.
EventGroupHandle_t xEventGroup;

TimerHandle_t xTimerSensor;
TimerHandle_t xTimerJoystick; /*timer for publish joystick/potentiometer*/
//TimerHandle_t xTimerHallSensor;
TimerHandle_t xTimerCalculateMotorSpeed;/*timer for publish the motor speed*/

uint32_t humidity_sensor = 50;
uint32_t Joystick_X = 90; /*initial value for joystick/motor*/
uint32_t hall_sensor_val = 0;/*inital value for hall sensor*/
uint32_t samples_cnt = 0;
bool sprinklers_on;
bool Motor_Push = false; /* Turn on/off motor*/
bool LEFT_MOTOR = false;/* Turn motor left*/
bool RIGHT_MOTOR = true;/* Turn motor right*/
bool flag_on_off = false;/* flag to know if the motor is on or off*/
double rpms = 0.0; /*rpm value for motor speed*/
int vueltas = 0;/*number of encoder activations*/
bool hallSensor = false; /*flag to know the hall sensor activation*/


/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Called when subscription request finishes.
 */
static void mqtt_topic_subscribed_cb(void *arg, err_t err)
{
    const char *topic = (const char *)arg;

    if (err == ERR_OK)
    {
        PRINTF("Subscribed to the topic \"%s\".\r\n", topic);
    }
    else
    {
        PRINTF("Failed to subscribe to the topic \"%s\": %d.\r\n", topic, err);
    }
}

/*!
 * @brief Called when there is a message on a subscribed topic.
 */
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    LWIP_UNUSED_ARG(arg);

    PRINTF("Received %u bytes from the topic \"%s\": \"", tot_len, topic);

}

/*!
 * @brief Called when recieved incoming published message fragment.
 */
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    int i;
    unsigned int value=200;

    LWIP_UNUSED_ARG(arg);

    for (i = 0; i < len; i++)
    {
        if (isprint(data[i]))
        {
            PRINTF("%c", (char)data[i]);
        }
        else
        {
            PRINTF("\\x%02x", data[i]);
        }
    }

    if(!memcmp(data, "RIGHT", 5)) { /*Turning Right/Left subscriber*/
    	sprinklers_on = true;
    	change_motor_turn();
    	xEventGroupSetBits(xEventGroup,	MQTT_SPRINKLERS_EVT);
    }
    else if(!memcmp(data, "LEFT", 4)) {/*Turning Right/Left subscriber*/
    	sprinklers_on = false;
    	change_motor_turn();
    	xEventGroupSetBits(xEventGroup,	MQTT_SPRINKLERS_EVT);
    }
    else if(!memcmp(data, "RELEASE", 7)){/*Turn on/off subscriber*/
    	toggle_motor();
    	xEventGroupSetBits(xEventGroup,	MQTT_SW_EVT);
    	//PRINTF("%s\"\r\n",data);
    }
    else if(!memcmp(data, "PUSH", 4)){/*Turn on/off subscriber*/

    	/*nada*/
    }
   else{/*speed_motor subscriber*/
    	value = atoi(data);
    	if((value >= 0) &&(value <= 100)){
    	  xQueueSendToBack(speed_por_queue, data, 0);
    	 xEventGroupSetBits(xEventGroup,MQTT_SET_SPEED_EVT);
     }

    }

    if (flags & MQTT_DATA_FLAG_LAST)
    {
        PRINTF("\"\r\n");
    }
}

/*!
 * @brief Subscribe to MQTT topics.
 */
static void mqtt_subscribe_topics(mqtt_client_t *client)
{
									/*Turnirn Right/Left			Turn on/off				Speed_motor*/
    static const char *topics[] = {"Eleusis_95/feeds/sprinkler","Eleusis_95/feeds/button","Eleusis_95/feeds/board"};
    int qos[]                   = {0,1,2};
    err_t err;
    int i;

    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb,
                            LWIP_CONST_CAST(void *, &mqtt_client_info));

    for (i = 0; i < ARRAY_SIZE(topics); i++)
    {
        err = mqtt_subscribe(client, topics[i], qos[i], mqtt_topic_subscribed_cb, LWIP_CONST_CAST(void *, topics[i]));

        if (err == ERR_OK)
        {
            PRINTF("Subscribing to the topic \"%s\" with QoS %d...\r\n", topics[i], qos[i]);
        }
        else
        {
            PRINTF("Failed to subscribe to the topic \"%s\" with QoS %d: %d.\r\n", topics[i], qos[i], err);
        }
    }
}

/*!
 * @brief Called when connection state changes.
 */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    const struct mqtt_connect_client_info_t *client_info = (const struct mqtt_connect_client_info_t *)arg;

    connected = (status == MQTT_CONNECT_ACCEPTED);

    switch (status)
    {
        case MQTT_CONNECT_ACCEPTED:
            PRINTF("MQTT client \"%s\" connected.\r\n", client_info->client_id);
            mqtt_subscribe_topics(client);
            xEventGroupSetBits(xEventGroup,	MQTT_CONNECTED_EVT);
            break;

        case MQTT_CONNECT_DISCONNECTED:
            PRINTF("MQTT client \"%s\" not connected.\r\n", client_info->client_id);
            /* Try to reconnect 1 second later */
            sys_timeout(1000, connect_to_mqtt, NULL);
            break;

        case MQTT_CONNECT_TIMEOUT:
            PRINTF("MQTT client \"%s\" connection timeout.\r\n", client_info->client_id);
            /* Try again 1 second later */
            sys_timeout(1000, connect_to_mqtt, NULL);
            break;

        case MQTT_CONNECT_REFUSED_PROTOCOL_VERSION:
        case MQTT_CONNECT_REFUSED_IDENTIFIER:
        case MQTT_CONNECT_REFUSED_SERVER:
        case MQTT_CONNECT_REFUSED_USERNAME_PASS:
        case MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_:
            PRINTF("MQTT client \"%s\" connection refused: %d.\r\n", client_info->client_id, (int)status);
            /* Try again 10 seconds later */
            sys_timeout(10000, connect_to_mqtt, NULL);
            break;

        default:
            PRINTF("MQTT client \"%s\" connection status: %d.\r\n", client_info->client_id, (int)status);
            /* Try again 10 seconds later */
            sys_timeout(10000, connect_to_mqtt, NULL);
            break;
    }
}

/*!
 * @brief Starts connecting to MQTT broker. To be called on tcpip_thread.
 */
static void connect_to_mqtt(void *ctx)
{
    LWIP_UNUSED_ARG(ctx);

    PRINTF("Connecting to MQTT broker at %s...\r\n", ipaddr_ntoa(&mqtt_addr));

    mqtt_client_connect(mqtt_client, &mqtt_addr, EXAMPLE_MQTT_SERVER_PORT, mqtt_connection_cb,
                        LWIP_CONST_CAST(void *, &mqtt_client_info), &mqtt_client_info);
}

/*!
 * @brief Called when publish request finishes.
 */
static void mqtt_message_published_cb(void *arg, err_t err)
{
    const char *topic = (const char *)arg;

    if (err == ERR_OK)
    {
        PRINTF("Published to the topic \"%s\".\r\n", topic);
    }
    else
    {
        PRINTF("Failed to publish to the topic \"%s\": %d.\r\n", topic, err);
    }
}

/*!
 * @brief Publishes a message. To be called on tcpip_thread.
 */
static void publish_humidity(void *ctx)
{
    static const char *topic   = "Eleusis_95/feeds/sensor-dummy";/**/
    static char message[10];

    LWIP_UNUSED_ARG(ctx);

    memset(message, 0, 10);
    sprintf(message, "%d", humidity_sensor);

    PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);

    mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 0, mqtt_message_published_cb, (void *)topic);
}
static void publish_Joystick(void *ctx){
	static const char *topic   = "Eleusis_95/feeds/potentiometer";/*Joystick_x/POT_100K publisher*/
	static char message[10];

	 LWIP_UNUSED_ARG(ctx);

	memset(message, 0, 10);
	sprintf(message, "%d", Joystick_X);
	PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);
	mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 0, mqtt_message_published_cb, (void *)topic);

}
static void publish_hall_sensor(void *ctx){

	static const char *topic   = "Eleusis_95/feeds/indicator";/*Hall sensor publisher*/
	static char message[10];

	LWIP_UNUSED_ARG(ctx);

	memset(message, 0, 10);
	sprintf(message, "%d", hall_sensor_val);
	PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);
	mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 0, mqtt_message_published_cb, (void *)topic);

}
static void publish_calculate_speed(void *ctx){

	static const char *topic   = "Eleusis_95/feeds/text";/*Velocidad del motor RPM publisher*/
	static char message[10];

	LWIP_UNUSED_ARG(ctx);

	memset(message, 0, 10);
	sprintf(message, "%d", (int)rpms);
	PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);
	mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 0, mqtt_message_published_cb, (void *)topic);

}

/*!
 * @brief Application thread.
 */
static void app_thread(void *arg)
{
    struct netif *netif = (struct netif *)arg;
    struct dhcp *dhcp;
    err_t err;
    const TickType_t xTicksToWait = 1000 / portTICK_PERIOD_MS;
    EventBits_t uxBits;
    uint32_t timerId = 0;
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };

    // Attempt to create the event group.
    xEventGroup = xEventGroupCreate();
    //Cinfigure servo
    servo_queue = xQueueCreate(10, MAX_CMD_LENGTH);
    speed_por_queue = xQueueCreate(10, MAX_CMD_LENGTH);/*creación de queue para velocidad de motor*/
    // Was the event group created successfully?
    if( xEventGroup == NULL ) {
    	PRINTF("Error creating the events group\r\n");
    }
    if (servo_queue != NULL)
    {
    	vQueueAddToRegistry(servo_queue, "servo");
    }
    if (speed_por_queue != NULL)
     {
        vQueueAddToRegistry(speed_por_queue, "speed");
     }

    xTimerSensor = xTimerCreate("TimerSns", 30000 / portTICK_PERIOD_MS, pdTRUE, (void*)&timerId, sensor_timer_callback);
    if( xTimerSensor == NULL ) {
    	PRINTF("Error creating the sensors timer\r\n");
    }
    xTimerJoystick = xTimerCreate("TimerSns", 50000 / portTICK_PERIOD_MS, pdTRUE, (void*)&timerId, joystick_stick_callback);/*Setting timer for MQTT_JOY_X_EVT event	*/
        if( xTimerJoystick == NULL ) {
        	PRINTF("Error creating the sensors timer\r\n");
        }
  /* xTimerHallSensor = xTimerCreate("Timerhall", 13000 / portTICK_PERIOD_MS, pdTRUE, (void*)&timerId, hall_sesnsor_callback);
         if( xTimerHallSensor == NULL ) {
            PRINTF("Error creating the sensors timer\r\n");
        }*/

   xTimerCalculateMotorSpeed = xTimerCreate("TimerCalSpeedMotor", 70000 / portTICK_PERIOD_MS, pdTRUE, (void*)&timerId, hall_motor_calculate_speed_callback);/*Setting timer for MQTT_MOTOR_READ_SPEED_EVT event	*/
   if( xTimerCalculateMotorSpeed == NULL ) {
      PRINTF("Error creating the sensors timer\r\n");
  }

    /* Init output LED GPIO. */
    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &led_config);
    /*Init motor turning on/off*/
    GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, &led_config);

   /*Init motor turning left/right*/
    GPIO_PinInit(BOARD_LEFT_MOTOR_GPIO, BOARD_LEFT_MOTOR_GPIO_PIN, &led_config);
    GPIO_PinInit(BOARD_RIGHT_MOTOR_GPIO, BOARD_RIGHT_MOTOR_GPIO_PIN, &led_config);
    /*initalitation for motor turning to left*/
    PIN_LEFT_MOTOR_ON();
    PIN_RIGHT_MOTOR_OFF();


    /*servo hall sensor*/
    /* Define the init structure for the input switch pin */
        gpio_pin_config_t sw_config = {
             kGPIO_DigitalInput,
                     0,
                 };
       /*initialitation for hall sensor interrupt*/
       #if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
           GPIO_SetPinInterruptConfig(BOARD_HALL_GPIO, BOARD_HALL_GPIO_PIN, kGPIO_InterruptFallingEdge);
       #else
           PORT_SetPinInterruptConfig(BOARD_HALL_PORT, BOARD_HALL_GPIO_PIN, kPORT_InterruptEitherEdge);
       #endif
           EnableIRQ(BOARD_HALL_S_IRQ);
           GPIO_PinInit(BOARD_HALL_GPIO, BOARD_HALL_GPIO_PIN, &sw_config);


    /*init motor speed*/
    init_motor_calculate_speed();

    /*set speed*/
    Init_set_motor_speed();
    /**/
    turn_off_pwm();
    /* Wait for address from DHCP */
    PRINTF("Getting IP address from DHCP...\r\n");

    do
    {
        if (netif_is_up(netif))
        {
            dhcp = netif_dhcp_data(netif);
        }
        else
        {
            dhcp = NULL;
        }

        sys_msleep(20U);

    } while ((dhcp == NULL) || (dhcp->state != DHCP_STATE_BOUND));

    PRINTF("\r\nIPv4 Address     : %s\r\n", ipaddr_ntoa(&netif->ip_addr));
    PRINTF("IPv4 Subnet mask : %s\r\n", ipaddr_ntoa(&netif->netmask));
    PRINTF("IPv4 Gateway     : %s\r\n\r\n", ipaddr_ntoa(&netif->gw));

    /*
     * Check if we have an IP address or host name string configured.
     * Could just call netconn_gethostbyname() on both IP address or host name,
     * but we want to print some info if goint to resolve it.
     */
    if (ipaddr_aton(EXAMPLE_MQTT_SERVER_HOST, &mqtt_addr) && IP_IS_V4(&mqtt_addr))
    {
        /* Already an IP address */
        err = ERR_OK;
    }
    else
    {
        /* Resolve MQTT broker's host name to an IP address */
        PRINTF("Resolving \"%s\"...\r\n", EXAMPLE_MQTT_SERVER_HOST);
        err = netconn_gethostbyname(EXAMPLE_MQTT_SERVER_HOST, &mqtt_addr);
    }

    if (err == ERR_OK)
    {
        /* Start connecting to MQTT broker from tcpip_thread */
        err = tcpip_callback(connect_to_mqtt, NULL);
        if (err != ERR_OK)
        {
            PRINTF("Failed to invoke broker connection on the tcpip_thread: %d.\r\n", err);
        }
    }
    else
    {
        PRINTF("Failed to obtain IP address: %d.\r\n", err);
    }

    while(1) {
		// Wait a maximum of 1s for either bit 0 or bit 4 to be set within
		// the event group.  Clear the bits before exiting.
		uxBits = xEventGroupWaitBits(
					xEventGroup,	// The event group being tested.
					MQTT_CONNECTED_EVT | MQTT_SENSOR_EVT | MQTT_SPRINKLERS_EVT | MQTT_DISCONNECTED_EVT |MQTT_SW_EVT | MQTT_JOY_X_EVT|MQTT_SET_SPEED_EVT|MQTT_HALL_SENSOR_EVT|MQTT_MOTOR_READ_SPEED_EVT,	// The bits within the event group to wait for.
					pdTRUE,			// BIT_0 and BIT_4 should be cleared before returning.
					pdFALSE,		// Don't wait for both bits, either bit will do.
					xTicksToWait );	// Wait a maximum of 100ms for either bit to be set.

		if(uxBits == 0) continue;


		if(uxBits & MQTT_CONNECTED_EVT ) {
			PRINTF("MQTT_CONNECTED_EVT.\r\n");
			//Start the sensor timer
			xTimerStart(xTimerSensor, 0);
			xTimerStart(xTimerJoystick,0);
			//xTimerStart(xTimerHallSensor,0);
			xTimerStart(xTimerCalculateMotorSpeed,0);
		}
		else if(uxBits & MQTT_SENSOR_EVT ) {
			PRINTF("MQTT_SENSOR_EVT.\r\n");
			// Simulate the humidity %, in steps of 5, range is 10% to 100%.
			// If the sprinkler is On, the humidity will tent to rise.
			humidity_sensor = get_simulated_sensor(humidity_sensor, 2, 10, 100, sprinklers_on);
			if((samples_cnt++%10) == 9){
				err = tcpip_callback(publish_humidity, NULL);
				if (err != ERR_OK)
				{
					PRINTF("Failed to invoke publish_humidity on the tcpip_thread: %d.\r\n", err);
				}
			}
		}
		else if(uxBits & MQTT_JOY_X_EVT){ /*detectio of MQTT_JOY_X_EVT event */
			PRINTF("MQTT_JOY_X_EVT.\r\n");
			Joystick_X = Joystick_position();/*function to get khoms from potentiometer*/
			//if((samples_cnt++%10) == 9){
				err = tcpip_callback(publish_Joystick, NULL);
				if (err != ERR_OK)
					{
						PRINTF("Failed to invoke Joystick_position on the tcpip_thread: %d.\r\n", err);
					}
			//	}


		}
		else if(uxBits & MQTT_SET_SPEED_EVT){/*detectio of MQTT_SET_SPEED_EVT event*/
				PRINTF("MQTT_SET_SPEED_EVT.\r\n");
				setting_speed();/*set speed to motor*/

				}
		else if(uxBits & MQTT_HALL_SENSOR_EVT){/*detectio of MQTT_HALL_SENSOR_EVT event */
			PRINTF("MQTT_HALL_SENSOR_EVT.\r\n");
			//hall_sensor_val = motor_active();
				//if((samples_cnt++%10) == 9){
			if(hallSensor){
				hall_sensor_val = 2;/*sen 2 if a magnetic field is detected*/
			}
			else{
				hall_sensor_val = 0;
			}
				err = tcpip_callback(publish_hall_sensor, NULL);
				if (err != ERR_OK)
					{
						PRINTF("Failed to invoke motor_active on the tcpip_thread: %d.\r\n", err);
					}
			//	}

				}
		else if(uxBits & MQTT_SPRINKLERS_EVT ) {/*detectio of MQTT_SPRINKLERS_EVT event */
			PRINTF("MQTT_SPRINKLERS_EVT.\r\n");
			if(sprinklers_on){
				GPIO_PortClear(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
			}
			else {
				GPIO_PortSet(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
			}
			if(LEFT_MOTOR){ /*set turn left */
				PIN_LEFT_MOTOR_ON();
				PIN_RIGHT_MOTOR_OFF();
			}
			else if(RIGHT_MOTOR){/*set turn right */
				PIN_LEFT_MOTOR_OFF();
				PIN_RIGHT_MOTOR_ON();
			}
		}
		else if(uxBits & MQTT_SW_EVT){/*detectio of MQTT_SW_EVT event */
			PRINTF("MQTT_SW_EVT.\r\n");
			if(Motor_Push){
				flag_on_off =!flag_on_off;
				GPIO_PortToggle(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN);
				if(flag_on_off){/*turn on motor*/
					turn_on_pwm();
				}
				else if(!flag_on_off){/*turn off motor*/
					turn_off_pwm();
				}

				Motor_Push = false;
				//LED_RED_ON();
			}
		}
		else if(uxBits & MQTT_MOTOR_READ_SPEED_EVT){/*detectio of MQTT_MOTOR_READ_SPEED_EVT event */
			PRINTF(" MQTT_MOTOR_READ_SPEED_EVT.\r\n");

			vueltas = motor_speed(); /*getting cuantit of interrupt activations*/
			rpms = calculate_speed(vueltas);/*calculeta the rpms*/
			err = tcpip_callback(publish_calculate_speed,NULL);
			if (err != ERR_OK)
			{
				PRINTF("Failed to invoke motor_active on the tcpip_thread: %d.\r\n", err);
				}
		}
		else if(uxBits & MQTT_DISCONNECTED_EVT ) {

			PRINTF("MQTT_DISCONNECTED_EVT.\r\n");
		}
		else
		{
			PRINTF("Un-known MQTT event\r\n");
		}
    }
    vTaskDelete(NULL);
}

/*!
 * @brief Main function
 */
int main(void)
{
    static struct netif fsl_netif0;
#if defined(FSL_FEATURE_SOC_LPC_ENET_COUNT) && (FSL_FEATURE_SOC_LPC_ENET_COUNT > 0)
    static mem_range_t non_dma_memory[] = NON_DMA_MEMORY_ARRAY;
#endif /* FSL_FEATURE_SOC_LPC_ENET_COUNT */
    ip4_addr_t fsl_netif0_ipaddr, fsl_netif0_netmask, fsl_netif0_gw;

    ethernetif_config_t fsl_enet_config0 = {
        .phyAddress = EXAMPLE_PHY_ADDRESS,
        .clockName  = EXAMPLE_CLOCK_NAME,
        .macAddress = configMAC_ADDR,
#if defined(FSL_FEATURE_SOC_LPC_ENET_COUNT) && (FSL_FEATURE_SOC_LPC_ENET_COUNT > 0)
        .non_dma_memory = non_dma_memory,
#endif /* FSL_FEATURE_SOC_LPC_ENET_COUNT */
    };

    SYSMPU_Type *base = SYSMPU;
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    /*Init ADC for Joystick*/
    Init_JoyStick();
    /* Disable SYSMPU. */
    base->CESR &= ~SYSMPU_CESR_VLD_MASK;
    /*hall_init*/
    //hall_init();
    /*Hallsensor*/

    /*motor speed*/
   // Init_speed_Control();

     NVIC_SetPriority(BOARD_HALL_S_IRQ, 3);/*definition of interrupt priority, this beacuse the interrupt priority has a higher priority that the sistem priority*/



    IP4_ADDR(&fsl_netif0_ipaddr, 0U, 0U, 0U, 0U);
    IP4_ADDR(&fsl_netif0_netmask, 0U, 0U, 0U, 0U);
    IP4_ADDR(&fsl_netif0_gw, 0U, 0U, 0U, 0U);

    tcpip_init(NULL, NULL);

    mqtt_client = mqtt_client_new();
    if (mqtt_client == NULL)
    {
        PRINTF("mqtt_client_new() failed.\r\n");
        return 1;
    }

    netifapi_netif_add(&fsl_netif0, &fsl_netif0_ipaddr, &fsl_netif0_netmask, &fsl_netif0_gw, &fsl_enet_config0,
                       ethernetif0_init, tcpip_input);
    netifapi_netif_set_default(&fsl_netif0);
    netifapi_netif_set_up(&fsl_netif0);

    netifapi_dhcp_start(&fsl_netif0);

    PRINTF("\r\n************************************************\r\n");
    PRINTF(" MQTT client example\r\n");
    PRINTF("************************************************\r\n");

    if (sys_thread_new("app_task", app_thread, &fsl_netif0, APP_THREAD_STACKSIZE, APP_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("main(): Task creation failed.", 0);
    }
    /*Servo task*/
    /*if (sys_thread_new("servo_task", servo_task, &fsl_netif0, APP_THREAD_STACKSIZE, APP_THREAD_PRIO - 1) == NULL)
       {
           LWIP_ASSERT("main(): Task creation failed.", 0);
       }*/

    vTaskStartScheduler();

    /* Will not get here unless a task calls vTaskEndScheduler ()*/
    return 0;
}

int32_t get_simulated_sensor(int32_t current_value, int32_t max_step, int32_t min_value, int32_t max_value, bool increase)
{
	uint32_t coin;
	int32_t step;
	// flip the coin to see if the value increases o decreases:
	coin = rand()%100;
	step = (int32_t)rand()%max_step;
	if(coin > (increase?30:70)){
		if((current_value + step) <= max_value) {
			current_value += step;
		}
		else {
			current_value = max_value;
		}
	}
	else {
		if((current_value - step) >= min_value) {
			current_value -= step;
		}
		else {
			current_value = min_value;
		}
	}
	return current_value;
}

void sensor_timer_callback( TimerHandle_t pxTimer )/*set MQTT_SENSOR_EVT event when timer is reached*/
{
	xEventGroupSetBits(xEventGroup,	MQTT_SENSOR_EVT);
}

/*********mine****************/
static void toggle_motor(void){ /*change Motor_Push flag*/
	Motor_Push =!Motor_Push;
}
static void change_motor_turn(void){/*change motor turning*/
	if(LEFT_MOTOR){
		LEFT_MOTOR = false;
		RIGHT_MOTOR = true;
	}
	else if(RIGHT_MOTOR){
		LEFT_MOTOR = true;
		RIGHT_MOTOR = false;
	}
}

static double calculate_speed(int vueltas){/*motor speed calculate*/
	vueltas = vueltas/2;
	rpms = (double)vueltas/20;
	rpms = rpms*60;
	return rpms;

}
/*Publisher*/
static void joystick_stick_callback(TimerHandle_t pxTimer){/* timer para joystick/potenciometer/temp_sensor*/
	xEventGroupSetBits(xEventGroup,MQTT_JOY_X_EVT);

}
/*static void hall_sesnsor_callback(TimerHandle_t pxTimer){/*calback function to handle timer to publish hall sensor state
	xEventGroupSetBits(xEventGroup,MQTT_HALL_SENSOR_EVT);
}*/
static void hall_motor_calculate_speed_callback(TimerHandle_t pxTimer){/*calback function to handle timer to publish */
	xEventGroupSetBits(xEventGroup,MQTT_MOTOR_READ_SPEED_EVT);
}
void BOARD_HALL_IRQ_HANDLER(void)/*interrupt for hall sensor*/
{
	 /* Change state of button. */
	    BaseType_t xHigherPriorityTaskWoken, xResult;
	    hallSensor = !hallSensor;
	      /* xHigherPriorityTaskWoken must be initialised to pdFALSE. */
	      xHigherPriorityTaskWoken = pdFALSE;
	      GPIO_PortClearInterruptFlags(BOARD_HALL_GPIO, 1U << BOARD_HALL_GPIO_PIN);
	      /* Set bit 0 and bit 4 in xEventGroup. */
	      xResult = xEventGroupSetBitsFromISR(
	                                  xEventGroup,   /* The event group being updated. */
									  MQTT_HALL_SENSOR_EVT, /* The bits being set. */
	                                  &xHigherPriorityTaskWoken );

    /* Clear external interrupt flag. */

    if( xResult != pdFAIL )
    		  {
    		      /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
    		      switch should be requested.  The macro used is port specific and will
    		      be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
    		      the documentation page for the port being used. */
    		      portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    		  }


/*#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif*/
}
#endif
