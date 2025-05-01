#!/usr/bin/env fish
# generated from catkin/cmake/template/setup.fish.in

# Sets various environment variables and sources additional environment hooks.
# It tries it's best to undo changes from a previously sourced setup file before.
# Supported command line options:
# --extend: skips the undoing of changes from a previously sourced setup file
# --local: only considers this workspace but not the chained ones
# In plain sh shell which doesn't support arguments for sourced scripts you can
# set the environment variable `CATKIN_SETUP_UTIL_ARGS=--extend/--local` instead.

# since this file is sourced either use the provided _CATKIN_SETUP_DIR
# or fall back to the destination set at configure time

if not type -q bass
    echo "Missing required fish plugin: bass. See https://github.com/edc/bass"
    exit 22
end

if test -z $_CATKIN_SETUP_DIR
    set _CATKIN_SETUP_DIR /usr/local
end

set _SETUP_UTIL "$_CATKIN_SETUP_DIR/_setup_util.py"
set -e _CATKIN_SETUP_DIR

if not test -f "$_SETUP_UTIL"
    echo "Missing Python script: $_SETUP_UTIL"
    exit 22
end

# detect if running on Darwin platform
set _UNAME (uname -s)
set _IS_DARWIN 0

if test "$_UNAME" = Darwin
    set _IS_DARWIN 1
end

set -e _UNAME

# make sure to export all environment variables
set -x CMAKE_PREFIX_PATH $CMAKE_PREFIX_PATH
if test $_IS_DARWIN -eq 0
    set -x LD_LIBRARY_PATH $LD_LIBRARY_PATH
else
    set -x DYLD_LIBRARY_PATH $DYLD_LIBRARY_PATH
end

set -e _IS_DARWIN
set -x PATH $PATH
set -x PKG_CONFIG_PATH $PKG_CONFIG_PATH
set -x PYTHONPATH $PYTHONPATH

# remember type of shell if not already set
if test -z "$CATKIN_SHELL"
    set CATKIN_SHELL fish
end

# invoke Python script to generate necessary exports of environment variables
# use TMPDIR if it exists, otherwise fall back to /tmp
if test -d "$TMPDIR"
    set _TMPDIR "$TMPDIR"
else
    set _TMPDIR /tmp
end

set _SETUP_TMP (mktemp "$_TMPDIR/setup.fish.XXXXXXXXXX")
set -e _TMPDIR

if test $status -ne 0 -o ! -f "$_SETUP_TMP"
    echo "Could not create temporary file: $_SETUP_TMP"
    exit 1
end

CATKIN_SHELL=$CATKIN_SHELL "$_SETUP_UTIL" "$argv" "$CATKIN_SETUP_UTIL_ARGS" >> "$_SETUP_TMP"
set _RC $status

if test $_RC -ne 0
    if test $_RC -eq 2
        then
        echo "Could not write the output of '$_SETUP_UTIL' to temporary file '$_SETUP_TMP': maybe the disk is full?"
    else
        echo "Failed to run '\"$_SETUP_UTIL\" $argv': return code $_RC"
    end
    set -e _RC
    set -e _SETUP_UTIL
    rm -f "$_SETUP_TMP"
    set -e _SETUP_TMP
    exit 1
end

set -e _RC
set -e _SETUP_UTIL
source "$_SETUP_TMP"
rm -f "$_SETUP_TMP"
set -e _SETUP_TMP

# source all environment hooks
set _i 0
while test $_i -lt $_CATKIN_ENVIRONMENT_HOOKS_COUNT
    # fish doesn't allow use of ${} to delimit variables within a string
    set _i_WORKSPACE (string join "" "$i" "_WORKSPACE")

    eval set _envfile \$_CATKIN_ENVIRONMENT_HOOKS_$_i
    set -e _CATKIN_ENVIRONMENT_HOOKS_$_i
    eval set _envfile_workspace \$_CATKIN_ENVIRONMENT_HOOKS_$_i_WORKSPACE
    set -e _CATKIN_ENVIRONMENT_HOOKS_$_i_WORKSPACE

    # set workspace for environment hook
    set CATKIN_ENV_HOOK_WORKSPACE $_envfile_workspace

    # non ideal: some packages register bash scripts as fish env hooks
    # it is needed to perform an extension check for backwards compatibility
    # if the script ends with .sh, .bash or .zsh, run it with bass
    set IS_SH_SCRIPT (string match -r '\.(sh|bash|zsh)$' "$_envfile")
    if test -n "$IS_SH_SCRIPT"
        bass source "$_envfile"
    else
        source "$_envfile"
    end

    set -e IS_SH_SCRIPT
    set -e CATKIN_ENV_HOOK_WORKSPACE
    set _i (math $_i + 1)
end
set -e _i

set -e _CATKIN_ENVIRONMENT_HOOKS_COUNT
