#!/bin/bash

echo 'begin to install requirements'
sleep 1

echo 'step 1: install pip3'
sudo apt-get install python3-pip

sleep 1

echo 'step 2: install requirements'
pip3 install -r requirements.txt
