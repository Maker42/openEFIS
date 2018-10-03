FROM postgres:10.5-alpine

# ensure local python is preferred over distribution python
ENV PATH /usr/local/bin:$PATH

# http://bugs.python.org/issue19846
# > At the moment, setting "LANG=C" on a Linux system *fundamentally breaks Python 3*, and that's not OK.
ENV LANG C.UTF-8

# if this is called "PIP_VERSION", pip explodes with "ValueError: invalid truth value '<VERSION>'"
ENV PYTHON_PIP_VERSION 18.0

ENV GPG_KEY 0D96DF4D4110E5C43FBFB17F2D347EA6AA65421D
ENV PYTHON_VERSION 3.6.6

# install ca-certificates so that HTTPS works consistently
# other runtime dependencies for Python are installed later

RUN set -ex \
    && apk add --no-cache --virtual .fetch-deps \
        ca-certificates \
        py3-psycopg2 \
    && wget -O get-pip.py 'https://bootstrap.pypa.io/get-pip.py'; \
    \
    python3 get-pip.py \
        --disable-pip-version-check \
        --no-cache-dir \
        "pip==$PYTHON_PIP_VERSION" \
    ; \
    pip --version; \
    \
    find /usr/local -depth \
        \( \
            \( -type d -a \( -name test -o -name tests \) \) \
            -o \
            \( -type f -a \( -name '*.pyc' -o -name '*.pyo' \) \) \
        \) -exec rm -rf '{}' +; \
    rm -f get-pip.py \
 && pip install pyyaml \
 && mkdir /usr/local/pgsql \
 && mkdir /usr/local/pgsql/data \
 && chown postgres /usr/local/pgsql/data \
 && su postgres -c "initdb -D /usr/local/pgsql/data" \
 && su postgres -c "pg_ctl start -D /usr/local/pgsql/data" \
 && su postgres -c "createdb flight" \
 && su postgres -c "kill -INT `head -1 /usr/local/pgsql/data/postmaster.pid`"

VOLUME /home/openEFIS:/home/openEFIS
USER postgres
WORKDIR /home/openEFIS/Autopilot
CMD ["./startDB.sh"]

