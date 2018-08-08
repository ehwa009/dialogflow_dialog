#!/usr/bin/env python
#-*- encoding: utf8 -*-

import os
import json

import rospkg
import rospy
import dialogflow

from mind_msgs.msg import Reply, RaisingEvents
from mind_msgs.srv import ReloadWithResult, ReadData, WriteData

class Dialog:
    def __init__(self):
        proj_file = os.path.join(
            rospkg.RosPack().get_path('dialogflow_dialog'), 'config', 'project_info.json')

        try:
            with open(proj_file) as f:
                info = json.loads(f.read())
                self.project_id = info["project_id"]
                self.language_code = info["language_code"]
        except IOError as e:
            rospy.logerr("Please, check the existance 'project.info.json' file in config...")
            exit()
        except AttributeError as e:
            rospy.logerr("Please, check the attribute for this error: %s"%e)
            exit()

        self.session_client = dialogflow.SessionsClient()
        self.session = self.session_client.session_path(self.project_id, 0)
        rospy.loginfo('Session path: {}'.format(self.session))

        self.pub_reply = rospy.Publisher('reply', Reply, queue_size=10)
        rospy.Subscriber('raising_events', RaisingEvents, self.handle_raise_events)

        rospy.loginfo('[%s] Initialzed'%rospy.get_name())

    def handle_raise_events(self, msg):
        if msg.recognized_word != '':
            text_input = dialogflow.types.TextInput(text=msg.recognized_word, language_code=self.language_code)
            query_input = dialogflow.types.QueryInput(text=text_input)
            response = self.session_client.detect_intent(session=self.session, query_input=query_input)

            reply_msg = Reply()
            reply_msg.header.stamp = rospy.Time.now()
            reply_msg.reply = response.query_result.fulfillment_text

            self.pub_reply.publish(reply_msg)
        else:
            event_name = "social_events_" + msg.events[0]
            event_input = dialogflow.types.EventInput(name=event_name, language_code=self.language_code)
            query_input = dialogflow.types.QueryInput(event=event_input)
            response = self.session_client.detect_intent(session=self.session, query_input=query_input)

            # Seperate unnessary event or unknow event with fallback intent

            reply_msg = Reply()
            reply_msg.header.stamp = rospy.Time.now()
            #print response
            reply_msg.reply = response.query_result.fulfillment_text

            self.pub_reply.publish(reply_msg)

if __name__ == '__main__':
    rospy.init_node('dialogflow_dialog', anonymous=False)
    m = Dialog()
    rospy.spin()
