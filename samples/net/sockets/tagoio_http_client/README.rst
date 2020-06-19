.. _sockets-tagoio-http-client-sample:

Socket TagoIO HTTP(S) Client
############################

Overview
********

This sample application implements an HTTP(S) client that will do an HTTP
or HTTPS post request to TagoIO IoT Service Platform. The sample sends
random temperature values to simulate a real device. This can be used to
speed-up development and shows how send simple JSON data to TagoIO servers.

The source code for this sample application can be found at:
:zephyr_file:`samples/net/sockets/tagoio_http_client`.

Requirements
************

- :ref:`networking_with_host`

Building and Running
********************

You can use this application on a supported board with networking or add a
shield like :ref:`esp_8266` to enable WIFI support.

Build the tagoio-http-client sample application like this:

.. zephyr-app-commands::
   :zephyr-app: samples/net/sockets/tagoio_http_client
   :board: sam4e_xpro
   :conf: <config file to use>
   :goals: build
   :compact:

Enabling TLS support
====================

Enable TLS support in the sample by building the project with the
``overlay-tls.conf`` overlay file enabled using these commands:

.. zephyr-app-commands::
   :zephyr-app: samples/net/sockets/tagoio_http_client
   :board: sam4e_xpro
   :conf: "prj.conf overlay-tls.conf"
   :goals: build
   :compact:

An alternative way is to specify ``-DOVERLAY_CONFIG=overlay-tls.conf`` when
running ``west build`` or ``cmake``.

The certificate and private key used by the sample can be found in the sample's
:zephyr_file:`samples/net/sockets/tagoio_http_client/src/` directory.
The default certificates used by Socket TagoIO HTTP(S) Client and
``https-server.py`` program found in the
`net-tools <https://github.com/zephyrproject-rtos/net-tools>`_ project, enable
establishing a secure connection between the samples.
