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

from std_msgs.msg import String
from flask import Flask, Response, request, make_response, jsonify

WEATHER_TEXT = [
    "The weather in {city} now is {current_weather_desc}. The temperature is {current_temp} degree and the wind speed is {current_wind_speed} m/s",
]

class WebhookServer:
    def __init__(self):
        self.app = Flask(__name__)
        self.app.add_url_rule('/', 'fulfillment', self.handle_fulfillment, methods=['POST'])
        self.app.add_url_rule('/', 'index', self.handle_index, methods=['GET'])

        self.port_num = rospy.get_param('~port_num', default=8888)
        try:
            with open(rospy.get_param('~weather_api')) as f:
                self.weather_api_key = json.loads(f.read())
        except KeyError, e:
            logging.error('Need parameter ~weather_api')
            exit(-1)

    def run(self):
        self.app.run(host="0.0.0.0", port=self.port_num)

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

        return make_response(jsonify({'fulfillmentText': res}))

    def get_weather(self, req):
        parameters = req.get('queryResult').get('parameters')

        result = requests.get('http://api.openweathermap.org/data/2.5/weather?q=%s&appid=%s'%(parameters['geo-city'], '5bb301572d2ee3362cd936bcf3d20c2c'))
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