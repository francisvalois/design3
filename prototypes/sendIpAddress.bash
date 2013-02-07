#!/bin/bash

api_dev_key='db301b23c6c83f2e30c7e6c8b732a81b'
api_user_key='2c035ce8673e879f004989c6378fc003'

#Récupération de tous les paste_key
echo "-----RÉCUPÉRATION DE TOUS LES PASTE"
data="api_option=list&api_user_key="$api_user_key"&api_dev_key="$api_dev_key"&api_results_limit=1000"
listing=($(curl -v -X POST -d  $data http://pastebin.com/api/api_post.php | grep -E "<paste_key>([0-9a-zA-Z])*</paste_key>"))

#Suppression de tous les paste du compte
echo "-----SUPPRESSION DE TOUS LES PASTE"
for i in "${listing[@]}"
do
	api_paste_key=$(echo $i | gawk 'match($0, /<paste_key>([0-9a-zA-Z]*)/, arr) {print arr[1]}')
	data="api_option=delete&api_user_key="$api_user_key"&api_dev_key="$api_dev_key"&api_paste_key="$api_paste_key
	curl -v -X POST -d  $data http://pastebin.com/api/api_post.php
done

#Récupération de l'addresse ip local
echo "-----RÉCUPÉRATION DE L'ADRESSE IP"
ipAddress=$(ifconfig wlan0 | grep 'inet adr:' | cut -d: -f2 | awk '{ print $1}')
echo $ipAddress

#L'adresse ip est envoyé sur pastebin
echo "-----ENVOIE DE LADRESSE IP SUR PASTEBIN"
api_paste_code=$ipAddress
paste_date=$(date '+_%x_%X')
paste_name="Design3IP"$paste_date
data="api_dev_key="$api_dev_key"&api_option=paste&api_paste_code="$api_paste_code"&api_user_key="$api_user_key"&api_paste_name="$paste_name 
echo $data
curl -v -X POST -d  $data http://pastebin.com/api/api_post.php
