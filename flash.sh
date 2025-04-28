#!/bin/bash

# Check if an argument was provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 /path/to/your/file.elf"
    exit 1
fi

# The path to the ELF file to be flashed
ELF_FILE="$1"

# Check if the specified file exists
if [ ! -f "$ELF_FILE" ]; then
    echo "Error: File not found - $ELF_FILE"
    exit 1
fi

# Store the output in a temporary file so that the gdb server does not print
# logs to the same console during gdb debugging. In case of an error, print the
# output and exit with an error code.
TMPFILE=$(mktemp)
JLinkGDBServerCLExe -device MIMX9352_M33 -if SWD -speed 1000 > "$TMPFILE" 2>&1 &
GDBSERVER_PID=$!
sleep 2
# Check for GDB Server start-up errors
if ! ps -p $GDBSERVER_PID > /dev/null || grep -q "Error" "$TMPFILE"; then
    echo "Error starting GDB Server:"
    cat "$TMPFILE"
    rm "$TMPFILE"
    exit 1
fi

gdb-multiarch $ELF_FILE \
              -ex "target remote localhost:2331" \
              -ex "load" \
              -ex "interrupt" \
              -ex "continue" &

# Get the PID of the last background process (GDB)
GDB_PID=$!

sleep 0.5
kill $GDB_PID
kill $GDBSERVER_PID

# Check the exit status of JLinkExe to see if it was successful
if [ $? -eq 0 ]; then
    echo "Flashing successful."
else
    echo "Flashing failed."
fi
