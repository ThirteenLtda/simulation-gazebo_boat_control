As instruções de Plugin Model Position Controller.pdf são a base. O plugin foi modificado (mais abaixo).

Adicionar o código abaixo dentro de um <model></model> em um sdf. 

~~~
<plugin name="ModelPositionController" filename="libgazebo_boat_control.so">
	<info filepath = "data/simulation.txt" timestep = "0.1" interpolate = "true" start_time=”0” / >
	<controller type = "pid" pP = "10000" pD = "100" pI = "0" rP = "10000" rD = "100" rI = "0" / >
</plugin>
~~~

USO


Modifique os ganhos de acordo com o corpo rígido que se deseja encontrar. Abaixo está a configuração usada para controlar um corpo rígido com inércias e massa de 5000000. Considere apenas os ganhos angulares porque os lineares não foram bem ajustados. No angular o erro encontra-se na faixa de 10e-4 para os dois métodos de controle.

      <controller type = 'pid' pP = '100000000' pD = '1000000' pI = '10000000' rP = '1000000000' rD = '10000000' rI = '10000000' usePrecalculatedForces = 'true'/>
ou
      <controller type = 'sliding' pk = '100000' pa = '10000' rk = '10000000' ra = '100' usePrecalculatedForces = 'true'/>


Execute o comando rock-gazebo ???/world.sdf. Substitua pelo path do sdf correto. O  "data/simulation.txt" é referente à pasta onde se executa o o comando rock-gazebo (não sei se poderia estar dentro da pasta build do plugin). DadosSimulação contêm alguns arquivos que vem dos xls de DadosOriginaisLula e são usáveis no plugin.


MATLAB

O plugin foi modificado com uma correção na classe do log e criando outro arquivo de log. Os dados do gazebo são salvos em data_log.txt. O arquivo teste.m consegue importar estes dados. Note que seriam necessários modicações neste script para que as velocidades e acelerações angulares pudem ser comparadas com as referências que estão em velocidades e acelerações da representação roll pitch yaw.

