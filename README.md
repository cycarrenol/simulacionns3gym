# TALLER NS-3

Repositorio con el la documentación del simulador para la asignatura ME

Para la ejecución de este proyecto, debe clonarse este repositorio en la carpeta scratch del simulador ns-3
previamente compilado, luego de eso se debe ejecutar el entorno de red con los siguientes comandos:

./waf

./waf --run "simulacionns3gym"

Esto se debe realizar un nivel arriba de la carpeta scratch, al mismo nivel del archivo waf, luego de esto en 
otra consola desde la carpeta simulacionns3gym se debe proceder a ejecutar el envirroment proxy para el 
agente OpenAI Gym con el siguiente comando:

./environmentProxy.py

Las respectivas consolas mostraran los resultados de la simulación.
