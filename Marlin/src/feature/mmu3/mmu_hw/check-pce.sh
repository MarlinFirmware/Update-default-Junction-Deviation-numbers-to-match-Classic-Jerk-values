#!/bin/bash

# download Prusa Error Codes for MMU
#wget https://raw.githubusercontent.com/3d-gussner/Prusa-Error-Codes/master/04_MMU/error-codes.yaml --output-document=error-codes.yaml
wget https://raw.githubusercontent.com/prusa3d/Prusa-Error-Codes/master/04_MMU/error-codes.yaml --output-document=error-codes.yaml

oifs="$IFS"  ## save original IFS
IFS=$'\n'    ## set IFS to break on newline
codes=($(cat error-codes.yaml |grep "code:" |cut -d '"' -f2))
titles=($(cat error-codes.yaml |grep 'title:' |cut -d '"' -f2))
texts=($(cat error-codes.yaml |grep "text:" |cut -d '"' -f2))
actions=($(cat error-codes.yaml |grep "action:" |cut -d ':' -f2))
ids=($(cat error-codes.yaml |grep "id:" |cut -d '"' -f2))
IFS="$oifs"  ## restore original IFS

filename=errors_list.h

clear
for ((i = 0; i < ${#codes[@]}; i++)) do
  code=${codes[i]}
  id=$(cat $filename |grep "${code#04*}" | cut -d "=" -f1 | cut -d "_" -f3- |cut -d " " -f1)
  title=$(cat $filename |grep "${id}" |grep --max-count=1 "MSG_TITLE" |cut -d '"' -f2)
  text=$(cat $filename |grep "${id}" |grep --max-count=1 "MSG_DESC" |cut -d '"' -f2)
  action1=$(cat $filename |grep "),//$id"| cut -d "," -f1)
  action2=$(cat $filename |grep "),//$id"| cut -d "," -f2)
  action1=$(echo $action1 | cut -d ":" -f2- |cut -d ":" -f2)
  action2=$(echo $action2 | cut -d ":" -f2- |cut -d ":" -f2 |cut -d ")" -f1)
  if [ "$action2" == "NoOperation" ]; then
  action=" [$action1]"
  else
  action=" [$action1,$action2]"
  fi
  echo -n "code: $code |"
  if [ "$id" != "${ids[i]}" ]; then
    echo -n "$(tput setaf 1) $id $(tput sgr0) # $(tput setaf 2)${ids[i]}$(tput sgr0)|"
  else
    echo -n " $id |"
  fi
  if [ "$title" != "${titles[i]}" ]; then
    echo -n "$(tput setaf 1) $title $(tput sgr0) # $(tput setaf 2)${titles[i]}$(tput sgr0)|"
  else
    echo -n " $title |"
  fi
  if [ "$text" != "${texts[i]}" ]; then
    echo -n "$(tput setaf 1) $text $(tput sgr0) # $(tput setaf 2)${texts[i]}$(tput sgr0)|"
  else
    echo -n " $text |"
  fi
  if [ "$action" != "${actions[i]}" ]; then
    echo -n "$(tput setaf 1) $action $(tput sgr0) # $(tput setaf 2)${actions[i]}$(tput sgr0)|"
  else
    echo -n " $action |"
  fi
  echo
done
