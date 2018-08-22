#!/usr/bin/env python
#-*- encoding: utf8 -*-

import os
import time
import rospy
import threading
import logging
import json
import requests
import random

from std_msgs.msg import String, Int16, Empty
from flask import Flask, Response, request, make_response, jsonify

WEATHER_TEXT = [
    "The weather in {city} now is {current_weather_desc}, current temperature is {current_temp} degree and wind speed is {current_wind_speed} m/s.",
]

class WebhookServer:
    def __init__(self):
        self.app = Flask(__name__)
        self.app.add_url_rule('/', 'fulfillment', self.handle_fulfillment, methods=['POST'])
        self.app.add_url_rule('/', 'index', self.handle_index, methods=['GET'])

        # 0: Neutral, 1: Forward Lean, 2: Self disclosure, 3: voice pitch
        self.current_scenario = 0 # Neutral
        rospy.Subscriber('/select_evaluation_scenario', Int16, self.handle_select_scenario)
        self.pub_complete = rospy.Publisher('/complete_execute_scenario', Empty, queue_size=1)

        self.port_num = rospy.get_param('~port_num', default=8888)
        try:
            with open(rospy.get_param('~weather_api')) as f:
                self.weather_api_key = json.loads(f.read())
        except KeyError, e:
            logging.error('Need parameter ~weather_api')
            exit(-1)

        # print self.weather_api_key

    def run(self):
        self.app.run(host="0.0.0.0", port=self.port_num)

    def handle_select_scenario(self, msg):
        self.current_scenario = msg.data

    def handle_index(self):
        return "<h1>This page is index page of UoA webhook server...</h1>"

    def handle_fulfillment(self):
        req = request.get_json(silent=True, force=True)
        try:
            action = req.get('queryResult').get('action')
        except AttributeError:
            rospy.logwarn('JSON error from fulfillment request')
            return "json error"

        if action == 'weather':
            res = self.get_weather(req)
        elif action == 'welcome':
            if self.current_scenario == 2: # 2: Self disclosure
                res = "Hi there, my name is Nao, the receptionist robot. I'm a little nervous about this task, but how may I help you?"
            else:
                res = "Hi there, my name is Nao, the receptionist robot. How may I help you?"
        elif action == "prescription_not_ready":
            if self.current_scenario == 3: # 3: voice pitch
                res = '''
                <prosody pitch="-15%"> I'm sorry Sam, your doctor has not yet written your prescription and so it is not ready for collection at the moment</prosody>.
                <prosody pitch="-15%"> However, I have sent a message to your doctor</prosody>.
                <prosody pitch="-15%"> Once the prescription has been written, someone will call you and let you know</prosody>.
                <prosody pitch="-15%"> Is there anything else I can help you with</prosody>?
                '''
            else:
                res = req.get('queryResult').get('fulfillmentText')
        elif action == "dontknow_doctor_name":
            if self.current_scenario == 2: # 2: Self disclosure
                res = '''
                No problem Sam, I forget things too sometimes.
                I can see that you have an appointment with Dr Jones today and have checked you in. Is there anything else I can help you with?
                '''
            elif self.current_scenario == 3: # 3: voice pitch
                res = '''
                <prosody pitch="10%"> No problem Sam, I can see that you have an appointment with Dr Jones today and have checked you in</prosody>.
                <prosody pitch="10%"> Is there anything else I can help you with</prosody>?
                '''
            else:
                res = req.get('queryResult').get('fulfillmentText')
        elif action == "request_bathroom":
            if self.current_scenario == 3: # 3: voice pitch
                res == '''
                <prosody pitch="10%"> Certainly, the bathroom is located down the hall, second door on the right</prosody>.
                '''
            else:
                res = req.get('queryResult').get('fulfillmentText')
        elif action == "goodbye":
            if self.current_scenario == 3: # 3: voice pitch
                res == '''
                <prosody pitch="10%"> I hope you have a nice day, Sam</prosody>.
                '''
            else:
                res = req.get('queryResult').get('fulfillmentText')
            self.pub_complete.publish()



        return make_response(jsonify({'fulfillmentText': res}))

    def get_weather(self, req):
        parameters = req.get('queryResult').get('parameters')

        result = requests.get('http://api.openweathermap.org/data/2.5/weather?q=%s&appid=%s'%(parameters['geo-city'], self.weather_api_key['api_key']))
        weather_data = json.loads(result.text)

        # print weather_data

        current_city = weather_data['name']
        current_weather = weather_data['weather'][0]['main']
        current_weather_desc = weather_data['weather'][0]['description']
        current_temp = weather_data['main']['temp'] - 273.15 # Kelvin to Celcius
        current_wind_speed = weather_data['wind']['speed']

        output_string = random.choice(WEATHER_TEXT)
        return output_string.format(city=current_city, current_weather_desc=current_weather_desc, current_temp=current_temp, current_wind_speed=current_wind_speed)


if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('webhook_server_node', disable_signals=True)).start()
    time.sleep(0.5)
    m = WebhookServer()
    m.run()
