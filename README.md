# fil_pilote
Ce projet est basé sur la programmation en language C de microcontroleur Microchip 16F1825.

Ce dispositif est une passerelle sans fils basé sur une communication 433MHz utilisant le protocole Home Easy entre un générateur de « fil pilote »  type programmateur d’un côté, et d’un ou plusieurs radiateurs électrique de l’autre.

Le but de ce dispositif est de piloter des radiateurs avec un programmateur hebdomadaire  en utilisant l’interface du « fil pilote », mais sans fil...
Dans une installation conventionnelle, le programmateur est relié physiquement aux différents radiateurs au travers d’un fil, dit « pilote » qui transmet l’ordre de commande: 

![Alt text](/Schema1.jpg)

Le dispositif se propose de remplacer cette liaison filaire par une liaison sans fils RF 433MHz:

![Alt text](/Schema2.jpg)

 Il est constitué de 2 boitiers 
-	Un boitier Tx qui sert de convertisseur ‘Fils Pilote -> Emetteur RF 433MHz’. Ce boitier est connecté
  localement au programmateur. Son rôle est de lire et décoder l’ordre « fil pilote » émanant  du 
  programmateur en local. Il diffuse ensuite cet ordre grâce à l’émetteur RF.
-	Un boitier Rx qui sert de convertisseur ‘Récepteur RF 433MHz -> Fils pilote’. Ce boitier est relié
  localement au niveau de chaque radiateur. Son rôle est de réceptionner et décoder la trame reçu en
  liaison RF. Il se charge ensuite de retranscrire l’ordre sur l’interface fils pilote local à 
  destination du radiateur sur lequel il est raccordé.

Le protocole Home Easy utilisé ici est exactement le même que celui des produits D-IO par Chacon.