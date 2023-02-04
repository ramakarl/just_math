#!/bin/bash
#
# To the extent possible under law, the person who associated CC0 with
# this file has waived all copyright and related or neighboring rights
# to this file.

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


####
# test for simple example and that it converges
# to the distribution we think it should
#

test_num=4
$bin -N $name_fn -R $rule_fn -X 4 -Y 3 -Z 2 -T $test_num | ./test4.py
res="$?"

if [[ "$res" != 0 ]] ; then
  echo "TEST $test_num FAILED: expected '1' got '$res'"
  exit -1
elif [[ "$verbosity" > 0 ]] ; then
  echo "# test $test_num passed"
fi


####
# test for simple example and that it converges
# to the distribution we think it should
#

test_num=5
$bin -N $name_fn -R $rule_fn -X 4 -Y 3 -Z 2 -T $test_num | ./test5.py
res="$?"

if [[ "$res" != 0 ]] ; then
  echo "TEST $test_num FAILED: expected '1' got '$res'"
  exit -1
elif [[ "$verbosity" > 0 ]] ; then
  echo "# test $test_num passed"
fi

####
# test for simple example and that it converges
# to the distribution we think it should
#
#
#test_num=12
#$bin -N $name_fn -R $rule_fn -X 4 -Y 3 -Z 2 -T $test_num | ./test12.py
#res="$?"
#
#if [[ "$res" != 0 ]] ; then
#  echo "TEST $test_num FAILED: expected '1' got '$res'"
#  exit -1
#elif [[ "$verbosity" > 0 ]] ; then
#  echo "# test $test_num passed"
#fi
#
#
#####
## test for simple example and that it converges
## to the distribution we think it should
##
#
#test_num=13
#$bin -N $name_fn -R $rule_fn -X 4 -Y 3 -Z 2 -T $test_num -S 0 | ./test13.py
#res="$?"
#
#if [[ "$res" != 0 ]] ; then
#  echo "TEST $test_num FAILED: expected '1' got '$res'"
#  exit -1
#elif [[ "$verbosity" > 0 ]] ; then
#  echo "# test $test_num passed"
#fi

####
# test full run
#

test_num=16
expect_res=`sha256sum bp_test16_output.txt | cut -f1 -d' '`
actual_res=`$bin -N $name_fn -R $rule_fn -X 4 -Y 3 -Z 2 -T $test_num -S 0 -V 1 | sha256sum | cut -f1 -d' '`
#res="$?"

if [[ "$expect_res" != "$actual_res" ]] ; then
  echo "TEST $test_num FAILED: expected sha256sum '$expect_res', got '$actual_res'"
  exit -1
elif [[ "$verbosity" > 0 ]] ; then
  echo "# test $test_num passed"
fi

test_num=17
$bin -N $name_fn -R $rule_fn -X 4 -Y 3 -Z 2 -T $test_num -S 0 -V 1 | ./test17
res="$?"

if [[ "$res" != 0 ]] ; then
  echo "TEST $test_num FAILED: expected '1', got '$res'"
  exit -1
elif [[ "$verbosity" > 0 ]] ; then
  echo "# test $test_num passed"
fi


exit 0
