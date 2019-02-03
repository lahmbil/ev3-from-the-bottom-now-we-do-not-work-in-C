#!/usr/bin/env bash

set -e # stop on first error

ev3_home="/home/bilal/ev3dev-c"
dist="/home/robot/ev3dev-c"
dir="${dist}/swagboy"

bin_name="swagboy"

ip="ev3dev.local"

cc_args="-DNDEBUG -Wall -W -std=gnu99 -O2 -DMEMWATCH -DMEMWATCH_STDIO"
args_lib="-lev3dev-c -lastar -pthread"

function check_return_value() {

    if [[ $? != 0 ]]; then

        echo "Error. Stopping process..."

        exit $?
    fi
}

function compile() {

    echo "Compiling..."

    docker run \
                --rm \
                -v ${ev3_home}:${dist-c}:z \
                -v ${PWD}/src:${dir}:z \
                -w ${dir} \
                --user $(id -u ${USER}):$(id -g ${USER}) \
                ev3ccs bash -c \
                    "arm-linux-gnueabi-gcc ${cc_args} -I ${dist}/source/ev3 -c *.c \
                     && \
                     arm-linux-gnueabi-g++ -o ${dir}/${bin_name} *.o -L${dist}/lib ${args_lib}" \
    &&
    rm -f ${PWD}/src/*.o \
    &&
    mv "$(pwd)/src/${bin_name}" "$(pwd)/bin/"

    check_return_value

    echo "Done"
}

function deploy() {

    echo "Deploying..."

    scp "$(pwd)/bin/swagboy" robot@${ip}:/home/robot/${bin_name}

    check_return_value

    echo "Done"
}

function run() {

    echo "Running..."

    ssh -tt robot@${ip} "time /home/robot/${bin_name}"

    echo "Done"
}

for i in $@; do

    if [[ "${i}" == "compile" ]]; then
     compile

    elif [[ "${i}" == "deploy" ]]; then
     deploy

    elif [[ "${i}" == "run" ]]; then
     run

    else
     echo "Unknown argument: ${i}"
    fi
done
