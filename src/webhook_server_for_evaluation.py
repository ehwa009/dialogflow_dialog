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
    "오늘 {city}의 날씨는 섭씨 {current_temp}도 이고 풍속은 {current_wind_speed} m/s 입니다. 꽤 선선한 날씨이네요."
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

        if action == 'intro_moon':
            res = "<expression=happiness:1.0>Hello! everyone? <br=1> <expression=happiness:0.9>Welcome to the Center for Automation and Robotic Engineering Science. <br=1>"
            res += "<expression=happiness:0.8>We are an interdisciplinary research hub. <br=0> \n"
            res += "<expression=happiness:1.0>My name is EveR. I am an android robot.  <br=1> \n"
            res += "<expression=happiness:0.7>It is so nice weather for looking around our lab. isn't it? <br=0> \n"
            res += "<expression=happiness:1.0>From now on, I will speak Korean to interact with you. <br=0> \n"
            res += "안녕하세요 문재인 대통령님, 저는 휴머노이드 로봇 에버입니다. <br=0> \n"
            res += "저희 연구실에 오신걸 진심으로 환영합니다! <br=0>"
            res += "<expression=happiness:0.7>대통령님께 제 모습을 보여드리기 위해서 많은 연습을 했답니다. <br=1>"
        elif action == 'face_kor':
            res = '''<expression=happiness:0.5>네 알겠어요. <br=1> \n
                    <expression=happiness:0.8>저는 여러분들과 감정 교류를 위해서 많은 표정을 지을수 있어요. <br=1>\n
                    <expression=happiness:0.5>바로 이렇게요. <br=0>
                    <expression=happiness:1.0> <br=4> \n
                    누군가 저를 화나게 한다면 이렇게요, <expression=anger:1.0> <br=4> \n
                    먼가 안좋은걸 봤을때는 이렇게요, <expression=disgust:1.0> <br=4> \n
                    무서울때는 이렇게요, <expression=fear:1.0> <br=4> \n
                    슬플때는, <expression=sadness:1.0> <br=4> \n
                    놀랐을때는, <expression=surprise:1.0> <br=4> \n
                    <expression=happiness:0.5> <br=3> 제 표정 어떠셧나요? 괜찮았나요? <br=1>
                    '''
        elif action == 'weather_eng':
            res = self.get_weather_eng(req)
        elif action == 'weather_kor':
            res = self.get_weather_kor(req)
        
        
        
        # elif action == 'welcome':
        #     if self.current_scenario == 2: # 2: Self disclosure
        #         res = "Hi there, my name is Nao, the receptionist robot. I'm a little nervous about this task, but how may I help you?"
        #     else:
        #         res = "Hi there, my name is Nao, the receptionist robot. How may I help you?"
        # elif action == "prescription_not_ready":
        #     if self.current_scenario == 3: # 3: voice pitch
        #         res = '''
        #         <prosody pitch="-15%"> I'm sorry Sam, your doctor has not yet written your prescription and so it is not ready for collection at the moment</prosody>.
        #         <prosody pitch="-15%"> However, I have sent a message to your doctor</prosody>.
        #         <prosody pitch="-15%"> Once the prescription has been written, someone will call you and let you know</prosody>.
        #         <prosody pitch="-15%"> Is there anything else I can help you with</prosody>?
        #         '''
        #     else:
        #         res = req.get('queryResult').get('fulfillmentText')
        # elif action == "dontknow_doctor_name":
        #     if self.current_scenario == 2: # 2: Self disclosure
        #         res = '''
        #         No problem Sam, I forget things too sometimes.
        #         I can see that you have an appointment with Dr Jones today and have checked you in. Is there anything else I can help you with?
        #         '''
        #     elif self.current_scenario == 3: # 3: voice pitch
        #         res = '''
        #         <prosody pitch="10%"> No problem Sam, I can see that you have an appointment with Dr Jones today and have checked you in</prosody>.
        #         <prosody pitch="10%"> Is there anything else I can help you with</prosody>?
        #         '''
        #     else:
        #         res = req.get('queryResult').get('fulfillmentText')
        # elif action == "request_bathroom":
        #     if self.current_scenario == 3: # 3: voice pitch
        #         res = '''
        #         %pointing=objects:door% <prosody pitch="10%"> Certainly, the bathroom is located down the hall, second door on the right</prosody>.
        #         '''
        #     else:
        #         res = req.get('queryResult').get('fulfillmentText')
        # elif action == "goodbye":
        #     if self.current_scenario == 3: # 3: voice pitch
        #         res = '''
        #         <prosody pitch="10%"> I hope you have a nice day, Sam</prosody>.
        #         '''
        #     else:
        #         res = req.get('queryResult').get('fulfillmentText')
        #     self.pub_complete.publish()

        return make_response(jsonify({'fulfillmentText': res}))

    def get_weather_eng(self, req):
        parameters = req.get('queryResult').get('parameters')
        # default city
        if len(parameters['geo-city'])<1:
            parameters = {
                'geo-city' : 'Auckland'
            }
        result = requests.get('http://api.openweathermap.org/data/2.5/weather?q=%s&appid=%s'%(parameters['geo-city'], self.weather_api_key['api_key']))
        weather_data = json.loads(result.text)

        # print weather_data

        current_city = weather_data['name']
        current_weather = weather_data['weather'][0]['main']
        current_weather_desc = weather_data['weather'][0]['description']
        current_temp = weather_data['main']['temp'] - 273.15 # Kelvin to Celcius
        current_wind_speed = weather_data['wind']['speed']

        output_string = WEATHER_TEXT[0]
        return output_string.format(city=current_city, current_weather_desc=current_weather_desc, current_temp=current_temp, current_wind_speed=current_wind_speed)

    def get_weather_kor(self, req):
        parameters = req.get('queryResult').get('parameters')
        # default city
        if len(parameters['geo-city'])<1:
            parameters = {
                'geo-city' : 'Auckland'
            }
        result = requests.get('http://api.openweathermap.org/data/2.5/weather?q=%s&appid=%s'%(parameters['geo-city'], self.weather_api_key['api_key']))
        weather_data = json.loads(result.text)

        # print weather_data

        current_city = weather_data['name']
        current_weather = weather_data['weather'][0]['main']
        current_weather_desc = weather_data['weather'][0]['description']
        current_temp = weather_data['main']['temp'] - 273.15 # Kelvin to Celcius
        current_wind_speed = weather_data['wind']['speed']

        output_string = WEATHER_TEXT[1]
        return output_string.format(city=current_city, current_temp=current_temp, current_wind_speed=current_wind_speed)


if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('webhook_server_node', disable_signals=True)).start()
    time.sleep(0.5)
    m = WebhookServer()
    m.run()
