#!/usr/bin/env bash

_robot_complete() {
    local cur prev
    cur="${COMP_WORDS[COMP_CWORD]}"

    local services="teleop rviz nav lifecycle thermal broadcaster decision thermocator thermulator sim map_save build"

    case "$COMP_CWORD" in
        1)
            COMPREPLY=($(compgen -W "$services" -- "$cur"))
            ;;
        2)
            COMPREPLY=($(compgen -W "38 0 1" -- "$cur"))
            ;;
        3)
            if [[ "${COMP_WORDS[1]}" == "build" ]]; then
                COMPREPLY=($(compgen -W "thermocator my_tb3_world" -- "$cur"))
            fi
            ;;
    esac
}

complete -F _robot_complete robot

_dock_complete() {
    local cur prev pprev
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    pprev="${COMP_WORDS[COMP_CWORD-2]}"

    local commands="start attach remote"
    local services="teleop rviz sim nav lifecycle thermal broadcaster decision launch stack build"

    case "$COMP_CWORD" in
        1)
            COMPREPLY=($(compgen -W "$commands" -- "$cur"))
            ;;
        2)
            if [[ "$prev" == "remote" ]]; then
                COMPREPLY=($(compgen -W "$services" -- "$cur"))
            fi
            ;;
        3)
            if [[ "$pprev" == "remote" && "$prev" == "build" ]]; then
                COMPREPLY=($(compgen -W "thermocator my_tb3_world" -- "$cur"))
            fi
            ;;
    esac
}

complete -F _dock_complete dock
