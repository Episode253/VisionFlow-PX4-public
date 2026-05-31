#!/usr/bin/env bash

WORLD="laboratory_landingbox"
ENTITY="q940_ti_0"
ENTITY_TYPE="MODEL"

AXIS="${1:-x}"

FORCE_MIN=5.0
FORCE_MAX=5.0

DURATION_MIN=1.0
DURATION_MAX=1.0

GAP_MIN=5.0
GAP_MAX=5.0

COUNT=10

rand_float() {
    local min=$1
    local max=$2
    awk -v min="$min" -v max="$max" -v seed="$RANDOM" \
        'BEGIN { srand(systime() + seed); printf "%.3f", min + rand() * (max - min) }'
}

clear_wrench() {
    gz topic -t /world/${WORLD}/wrench/clear \
        -m gz.msgs.Entity \
        -p "name: '${ENTITY}', type: ${ENTITY_TYPE}" >/dev/null 2>&1
}

apply_wrench_axis() {
    local value=$1

    local fx=0.0
    local fy=0.0
    local fz=0.0

    case "$AXIS" in
        x|X)
            fx="$value"
            ;;
        y|Y)
            fy="$value"
            ;;
        z|Z)
            fz="$value"
            ;;
        *)
            echo "[ERROR] Unknown axis: $AXIS"
            echo "Usage: ./random_wrench_axis.sh x|y|z"
            exit 1
            ;;
    esac

    gz topic -t /world/${WORLD}/wrench/persistent \
        -m gz.msgs.EntityWrench \
        -p "entity: {name: '${ENTITY}', type: ${ENTITY_TYPE}},
            wrench: {
              force: {x: ${fx}, y: ${fy}, z: ${fz}},
              torque: {x: 0.0, y: 0.0, z: 0.0}
            }"
}

trap 'echo ""; echo "[INFO] Interrupted. Clearing wrench..."; clear_wrench; exit 0' INT TERM

echo " World       : ${WORLD}"
echo " Entity      : ${ENTITY}"
echo " Entity type : ${ENTITY_TYPE}"
echo " Force range : ${FORCE_MIN} ~ ${FORCE_MAX} N"
echo " Duration    : ${DURATION_MIN} ~ ${DURATION_MAX} s"
echo " Gap         : ${GAP_MIN} ~ ${GAP_MAX} s"
echo " Count       : ${COUNT}"

for ((i=1; i<=COUNT; i++)); do
    force=$(rand_float "$FORCE_MIN" "$FORCE_MAX")
    duration=$(rand_float "$DURATION_MIN" "$DURATION_MAX")
    gap=$(rand_float "$GAP_MIN" "$GAP_MAX")

    if (( RANDOM % 2 == 0 )); then
        value="$force"
    else
        value="-$force"
    fi

    echo "[${i}/${COUNT}] Apply F${AXIS} = ${value} N, duration = ${duration} s"

    apply_wrench_axis "$value"
    sleep "$duration"

    clear_wrench
    echo "        Clear wrench, gap = ${gap} s"
    sleep "$gap"
done

clear_wrench
