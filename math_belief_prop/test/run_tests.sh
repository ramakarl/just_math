#!/bin/bash

verbosity=1

bin="../bpc.debug"
name_fn="./stair_name.csv"
rule_fn="./stair_rule.csv"

####
# test for basic realization
#

test_num=0
test_str="4,3,1"
res=`$bin -N $name_fn -R $rule_fn -X 4 -Y 3 -Z 2 -T $test_num | \
  head -n1 | grep -o "$test_str"`

if [[ "$res" != "$test_str" ]] ; then
  echo "TEST $test_num FAILED: expected '$test_str' got '$res'"
  exit -1
elif [[ "$verbosity" > 0 ]] ; then
  echo "# test $test_num passed"
fi


####
# test for basic filtration
#

test_num=1
$bin -N $name_fn -R $rule_fn -X 4 -Y 3 -Z 2 -T $test_num | ./test1.py
res="$?"

if [[ "$res" != 0 ]] ; then
  echo "TEST $test_num FAILED: expected '1' got '$res'"
  exit -1
elif [[ "$verbosity" > 0 ]] ; then
  echo "# test $test_num passed"
fi

####
# test for basic filtration
#

test_num=2
$bin -N $name_fn -R $rule_fn -X 4 -Y 3 -Z 2 -T $test_num | ./test2.py
res="$?"

if [[ "$res" != 0 ]] ; then
  echo "TEST $test_num FAILED: expected '1' got '$res'"
  exit -1
elif [[ "$verbosity" > 0 ]] ; then
  echo "# test $test_num passed"
fi


exit 0
