#!/usr/bin/env bash

_robot_complete() {
    local cur prev pprev
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    local services="bringup teleop rviz nav lifecycle thermal broadcaster decision \
                    thermocator thermulator map_save sim advisory pose_sync delta_thermal \
                    bridge build build_dt"

    case "$COMP_CWORD" in
        1)
            COMPREPLY=($(compgen -W "$services" -- "$cur"))
            ;;
        2)
            case "${COMP_WORDS[1]}" in
                bringup)
                    ;;
                build|build_dt)
                    COMPREPLY=($(compgen -W "thermocator my_tb3_world" -- "$cur"))
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
                build|build_dt)
                    ;;
                *)
                    COMPREPLY=($(compgen -W "38 0 1" -- "$cur"))
                    ;;
            esac
            ;;
        4)
            case "${COMP_WORDS[1]}" in
                build|build_dt)
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

    local commands="start attach remote"
    local services="teleop rviz sim nav lifecycle thermal broadcaster decision \
                    thermocator thermulator build build_dt bridge advisory pose_sync delta_thermal dt"

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
                    build|build_dt)
                        COMPREPLY=($(compgen -W "thermocator my_tb3_world" -- "$cur"))
                        ;;
                esac
            fi
            ;;
    esac
}
complete -F _dock_complete dock
