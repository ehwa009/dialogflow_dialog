# dialogflow_dialog
The dialog package for social_mind using dialogflow


## Installation

### Requirements

- Install Nodejs

	$ curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
	$ sudo apt install -y nodejs

- Install **localtunnel** (https://github.com/localtunnel/localtunnel)

        $ sudo npm install -g localtunnel

- Get account from https://openweathermap.org and get api_key, save weather_api.json in config

- Get Dialogflow account from https://console.dialogflow.com and save the project_info.json in config

- Get Service account from https://console.cloud.google.com ans dave the service_key.json in config


### Install clients

- Remove prior installation files for updates

        $ sudo rm -rf /usr/lib/python2.7/dist-packages/pyasn1_modules*

- Install DialogFlow client

        $ sudo pip install -U -r requirements.txt


## Usage

        $ roslaunch dialogflow_dialog bringup.launch lt_subdomain:=<your subdomain-name> lt_port:=<your port number>
