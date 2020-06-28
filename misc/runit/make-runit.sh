#!/bin/sh

OUTPUT=$1
BINDIR=$2
CONFDIR=$3
LIBDIR=$4

rm -f $OUTPUT

echo "#!/bin/sh" >> $OUTPUT
echo >> $OUTPUT
echo "exec su -c \"LD_LIBRARY_PATH=${LIBDIR} ${BINDIR}/rrr ${CONFDIR}\" - daemon" >> $OUTPUT