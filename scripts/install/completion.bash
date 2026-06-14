#!/usr/bin/env bash

_robot_complete() {
    local cur prev pprev
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    local services="bringup teleop rviz nav lifecycle thermal broadcaster decision \
                    thermocator thermulator arbiter map_save sim twin battery \
                    bridge build"

    case "$COMP_CWORD" in
        1)
            COMPREPLY=($(compgen -W "$services" -- "$cur"))
            ;;
        2)
            case "${COMP_WORDS[1]}" in
                bringup)
                    ;;
                build)
                    COMPREPLY=($(compgen -W "thermocator_msgs thermocator my_tb3_world" -- "$cur"))
                    ;;
                *)
                    ;;
            esac
            ;;
        3)
            case "${COMP_WORDS[1]}" in
                bringup)
                    COMPREPLY=($(compgen -W "38 0 1" -- "$cur"))
                    ;;
                build)
                    ;;
                *)
                    COMPREPLY=($(compgen -W "38 0 1" -- "$cur"))
                    ;;
            esac
            ;;
        4)
            case "${COMP_WORDS[1]}" in
                build)
                    # no-op
                    ;;
                *)
                    # package at position 4
                    COMPREPLY=($(compgen -W "thermocator my_tb3_world" -- "$cur"))
                    ;;
            esac
            ;;
    esac
}
complete -F _robot_complete robot

_dock_complete() {
    local cur prev pprev
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    pprev="${COMP_WORDS[COMP_CWORD-2]}"

    local commands="setup start attach remote"
    local services="teleop rviz sim nav lifecycle thermal broadcaster decision \
                    thermocator thermulator arbiter build bridge twin battery dt"

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
            if [[ "$pprev" == "remote" ]]; then
                case "$prev" in
                    build)
                        COMPREPLY=($(compgen -W "thermocator_msgs thermocator my_tb3_world" -- "$cur"))
                        ;;
                esac
            fi
            ;;
    esac
}
complete -F _dock_complete dock

