##################################################################################
#                                                                                #
#                          example_socket_select                                 #
#                                                                                #
##################################################################################

Table of Contents
~~~~~~~~~~~~~~~~~
 - Description
 - Setup Guide
 - Result description
 - Supported List
 
Description
~~~~~~~~~~~
	This example shows how to use socket select() to handle socket read from clients or remote server.
	
Setup Guide
~~~~~~~~~~~
        1. Modify SERVER_PORT definition for listen port of created TCP server.
        2. Can enable CONNECT_REMOTE to include TCP connection to remote server in example. 
        3. Modify REMOTE_HOST and REMOTE_PORT for remote server.
        4.GCC: use CMD "make all EXAMPLE=socket_select" to compile socket_select example.

      Can make automatical Wi-Fi connection when booting by using wlan fast connect example.
      
Result description
~~~~~~~~~~~~~~~~~~
       The socket select example thread will be started automatically when booting.
       A local TCP server will be started to wait for connection. Can use a TCP client connecting to this server to send data.
       If CONNECT_REMOTE is enabed in example. A remote TCP server is required and can send data to the created remote connection.

Supported List
~~~~~~~~~~~~~~
[Supported List]
        Supported IC :
               RTL8730A, RTL872XE