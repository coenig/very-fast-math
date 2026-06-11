# Call from vfm dir as "./show .. 10" (the 10 being optional) or from one level higher simply by "*vfm*/show.bash"

reset;
clear;

show () {
   echo "$1                    ==> last $2 lines"
   cat $1 | tail -n $2;
   echo ''
}

num=${2:-10}

names=`ls -d ./$1/*`
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

