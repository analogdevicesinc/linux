#!/bin/bash

V=${V:=false}
scriptName=${0##*/}
scriptDir="$(dirname $(readlink -f $0))"
mfile=$scriptDir/../MAINTAINERS.NXP
pl_script="./scripts/get_maintainer.pl"
maillist="\
linux-devel@linux.nxdi.nxp.com (Linux Factory Review List)
lnxrevli@nxp.com (i.MX Review List)"

usage() {
cat <<EOF
Wrapper script for kernel get_maintainer.pl

Usage: $0 [options] patchfile
       $0 [options] -f file|directory

Options:
  -s,--sendemail => send patch to maintainers for review
    --from=<address> => email sender
    --to=<address>,... => primary email recipient
    --cc=<address>,... => additional email recipient
    --confirm=<mode> => confirm just before sending (default: always)
  --help => show this help information

  Run "$pl_script -h" to get all other supported options.

EOF
}

args="$@"
# check -h and --help options
if [ -n "`echo $args |awk '/(^| )(-h|--help)( |$)/'`" ];then
    usage
    exit
fi

# filter out --sendemail arguments
sendemail=false
if echo $args |grep -Eq '(^| )(\-s|\-\-sendemail)( |$)';then
    sendemail=true
    $V && echo "Sendemail: $sendemail"
    args="`echo $args |sed -r 's,(^| )(-s|--sendemail)( |$), ,g'`"
fi

# filter out --from arguments
email_from=""
if echo $args |grep -Eq '\-\-from[ =]*[^ ]+';then
    email_from="`echo $args |sed 's,--from[ =]*[^ ]*,\n&,g;s,[^\n]*\n\(--from[ =]*[^ ]*\)[^\n]*,\1 ,g;s/.$//'`"
    $V && echo "From: `echo $email_from |sed 's,--from[ =]*,,g'`"
    args="`echo $args |sed -e 's,--from[ =]*[^ ]*,,g'`"
fi

# filter out --to arguments
email_to=""
if echo $args |grep -Eq '\-\-to[ =]*[^ ]+';then
    email_to="`echo $args |sed 's,--to[ =]*[^ ]*,\n&,g;s,[^\n]*\n\(--to[ =]*[^ ]*\)[^\n]*,\1 ,g;s/.$//'`"
    $V && echo "To: `echo $email_to |sed 's,--to[ =]*,,g'`"
    args="`echo $args |sed -e 's,--to[ =]*[^ ]*,,g'`"
fi

# filter out --cc arguments
email_cc=""
if echo $args |grep -Eq '\-\-cc[ =]*[^ ]+';then
    email_cc="`echo $args |sed 's,--cc[ =]*[^ ]*,\n&,g;s,[^\n]*\n\(--cc[ =]*[^ ]*\)[^\n]*,\1 ,g;s/.$//'`"
    $V && echo "Cc: `echo $email_cc |sed 's,--cc[ =]*,,g'`"
    args="`echo $args |sed -e 's,--cc[ =]*[^ ]*,,g'`"
fi

# filter out --confirm arguments
email_confirm="--confirm=always"
if echo $args |grep -Eq '\-\-confirm[ =]*[^ ]+';then
    email_confirm="`echo $args |sed 's,--confirm[ =]*[^ ]*,\n&,g;s,[^\n]*\n\(--confirm[ =]*[^ ]*\)[^\n]*,\1 ,g;s/.$//'`"
    $V && echo "Confirm: `echo $email_confirm |sed 's,--confirm[ =]*,,g'`"
    args="`echo $args |sed -e 's,--confirm[ =]*[^ ]*,,g'`"
fi

# remaining arguments to get_maintainer.pl
$V && echo "Arguments: $args"

# customized options to get_maintainer.pl
opts="--n --rolestats --no-git-fallback"

# filters for emails
filter="/@/{/@[^@]*(nxp.com|nxp1.onmicrosoft.com)/I!d;}"

perl $pl_script $opts --mpath $mfile $args |sed -r "$filter"
echo "$maillist"
echo

if $sendemail;then
   opts="--no-n --no-rolestats --no-git-fallback"
   emails="`perl $pl_script $opts --mpath $mfile $args |sed -r "$filter"`"
   emails="`echo $emails |sed -r 's,^[ \t]*,,;s,[ \t]*$,,;s/[ \t]+/,/g'`"
   $V && echo "Maintainers: $emails"
   # get patch files
   for arg in $args;do
       $V && echo $arg
       if [ -e "$arg" ] && [ -z "`git ls-files $arg`" ];then
           patches="$patches $arg"
       fi
   done
   $V && echo "Patches: $patches"
   if [ -z "$patches" ];then
       echo "Error: no patch file found."
       exit
   fi
   # display email sending command first in case users want to modify
   echo git send-email --no-chain-reply-to --no-signed-off-by-cc --quiet --suppress-cc=all $email_confirm $email_from --to=$emails $email_to $email_cc --cc=linux-devel@linux.nxdi.nxp.com,lnxrevli@nxp.com $patches
   git send-email --no-chain-reply-to --no-signed-off-by-cc --quiet --suppress-cc=all $email_confirm $email_from --to=$emails $email_to $email_cc --cc=linux-devel@linux.nxdi.nxp.com,lnxrevli@nxp.com $patches
fi
