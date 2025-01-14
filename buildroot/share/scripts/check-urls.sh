#!/usr/bin/env bash
#
# Check for broken URLs in Marlin files
#
[ -d "Marlin/src" ] || { echo "Run this script from the Marlin project folder!" ; exit 1 ; }

UA="Mozilla/5.0 (Linux; Android 10; SM-G996U Build/QP1A.190711.020; wv) AppleWebKit/537.36 (KHTML, like Gecko) Version/4.0 Mobile Safari/537.36"
UTMP=$(mktemp)
#echo "[debug 1] UTMP = ${UTMP}"
echo "Gathering URLs. Please wait..."
grep -R -E "https?:\/\/[^ \"''\(\)\<\>]+" . 2>/dev/null \
  | grep -v "Binary file" \
  | sed -E "s/\/https?:\/\//\//" \
  | sed -E 's/.*\((https?:\/\/[^ ]+)\).*$/\1/' \
  | sed -E 's/.*\[(https?:\/\/[^ ]+)\].*$/\1/' \
  | sed -E 's/.*(https?:\/\/[^ \"''()<>]+).*/\1/' \
  | grep -vE "(127\.0\.0\.1|localhost|myserver|doc\.qt\.io|docs\.google\.com|raw\.githubusercontent\.com|[\${}])" \
  | sed -E 's/]$//' | sed -E "s/'$//" | sed -E "s/[#.',]+$//" \
  | sed -E 's/youtu\.be\/(.+)/www.youtube.com\/watch?v=\1/' \
  | sort -u -R \
  >"$UTMP"

  #echo "[debug 2] link count = $(wc -l $UTMP)"
  ISERR=
  declare -a BADURLS
  while IFS= read -r URL
  do
    #echo -n "Checking ${URL} ... "
    HEAD=$(curl -s -I -A "${UA}" --request GET "${URL}" 2>/dev/null) ; HERR=$?
    if [[ $HERR > 0 ]]; then
      # Error 92 may be domain blocking curl / wget
      [[ $HERR == 92 ]] || { ISERR=1 ; BADURLS+=($URL) ; }
      echo "[FAIL ($HERR)]"
    else
      HEAD1=$(echo $HEAD | head -n1)
      EMSG=
      WHERE=
      case "$HEAD1" in
        *" 301"*) EMSG="[Moved Permanently]" ; WHERE=1 ;;
        *" 302"*) EMSG="[Moved Temporarily]" ; WHERE=1 ;;
        *" 303"*) echo "[See Other]" ;;
        *" 400"*) EMSG="[Invalid Request]" ;;
        *" 403"*) EMSG="[Forbidden]" ;;
        *" 404"*) EMSG="[Not Found]" ;;
        *" 503"*) EMSG="[Unavailable]" ;;
        *" 200"*) echo "[ OK ]" ;;
               *) EMSG="[Other Err]" ;;
      esac
      if [[ -n $EMSG ]]; then
        if [[ -n $WHERE ]]; then
          [[ ${HEAD,,} =~ "location: " ]] && EMSG+=" to $(echo "$HEAD" | grep -i "location: " | sed -E 's/location: (.*)/\1/')"
        else
          ISERR=1 ; BADURLS+=($URL)
        fi
        echo $EMSG
      fi
    fi
  done <"$UTMP"

  #echo "[debug 3]"
  if [[ -n $ISERR ]]; then
    # Join bad URLs into a bulleted markdown list
    printf -v BADSTR -- "- %s\n" "${BADURLS[@]}"
    echo -e "\nURL Checker reports one or more URLs could not be reached:\n${BADSTR}"
    exit 1
  fi

  echo -e "\nURL Check Passed."
  exit 0
