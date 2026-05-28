# Call from one level higher than vfm dir.

reset;
clear;

show () {
   echo "$1                    ==> last $2 lines"
   cat $1 | tail -n $2;
   echo ''
}

num=${1:-10}

names=`ls -d *`
for eachfile in $names
do
   FILE1=$eachfile/examples
   
   if [ -d $FILE1 ]; then
      subnames=`ls -d $FILE1/*`
      for eachsubfile in $subnames
      do
         FILE=$eachsubfile/results.txt
         if [ -f $FILE ]; then
            (show "$FILE" $num)
         fi
      done
   fi
done

