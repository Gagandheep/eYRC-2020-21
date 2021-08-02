import requests
import paho.mqtt.client as mqtt  # import the client1
import time


def iot_func_callback_sub(client, userdata, message):
    """Subscriber callback function

    Args:
       client (ActionClient): ROS action client
       userdata (userdata): user data
       message: ROS message
    """

    print("message received ", str(message.payload.decode("utf-8")))
    print("message topic=", message.topic)
    print("message qos=", message.qos)
    print("message retain flag=", message.retain)


def mqtt_subscribe_thread_start(arg_callback_func, arg_broker_url,
                                arg_broker_port, arg_mqtt_topic, arg_mqtt_qos):
    """Function to start a subscriber to listen to the MQTT topic

    Args:
       arg_callback_func (function): callback function for subscriber
       arg_broker_url (str): MQTT broker URL
       arg_broker_port (int): MQTT broker port
       arg_mqtt_topic (str): MQTT topic to subscribe
       arg_mqtt_qos (int): MQTT QOS
    """

    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1)  # wait
        # mqtt_client.loop_forever() # starts a blocking infinite loop
        mqtt_client.loop_start()  # starts a new thread
        return 0
    except:
        return -1


def mqtt_publish(arg_broker_url, arg_broker_port, arg_mqtt_topic,
                 arg_mqtt_message, arg_mqtt_qos):
    """Function to publish to an MQTT topic

    Args:
       arg_broker_url (str): MQTT broker URL
       arg_broker_port (int): MQTT broker port
       arg_mqtt_topic (str): MQTT topic to subscribe
       arg_mqtt_message (str): Message to send
       arg_mqtt_qos (int): MQTT QOS
    """

    try:
        mqtt_client = mqtt.Client("mqtt_pub")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.loop_start()

        print("Publishing message to topic", arg_mqtt_topic)
        mqtt_client.publish(arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos)
        time.sleep(0.1)  # wait

        mqtt_client.loop_stop()  # stop the loop
        return 0
    except:
        return -1


def push_data_to_sheet(webapp_id, **parameters):
    """Function to publish data to Google Sheet

    Args:
        webapp_id (str): google web-app ID
    """

    url = 'https://script.google.com/macros/s/' + webapp_id + '/exec'
    # Send request to publish to sheet
    response = requests.get(url, params=parameters)
    # Print response
    print(response.content)
