Projet fil pilote en 433MHz
===========================
### Description:

Ce projet est basé sur la programmation en language C de microcontrôleur Microchip [PIC16F1825][1].

Ce dispositif est une passerelle sans fil basée sur une communication 433MHz utilisant le protocole Home Easy entre un générateur de « fil pilote »  type programmateur d’un côté, et d’un ou plusieurs radiateurs électriques de l’autre.

Le but de ce dispositif est de piloter des radiateurs avec un programmateur hebdomadaire  en utilisant l’interface du « fil pilote », mais sans fil...
Dans une installation conventionnelle, le programmateur est relié physiquement aux différents radiateurs au travers d’un fil, dit « pilote » qui transmet l’ordre de commande: 

![Alt text](/Schema1.jpg)

Le dispositif se propose de remplacer cette liaison filaire par une liaison sans fil RF 433MHz:

![Alt text](/Schema2.jpg)

 Il est constitué de 2 boitiers 
 * Un boitier Tx qui sert de convertisseur ‘Fil Pilote -> Emetteur RF 433MHz’. Ce boitier est connecté localement au programmateur. Son rôle est de lire et décoder l’ordre « fil pilote » émanant  du programmateur en local. Il diffuse ensuite cet ordre grâce à l’émetteur RF.
 * Un boitier Rx qui sert de convertisseur ‘Récepteur RF 433MHz -> Fils pilote’. Ce boitier est relié localement au niveau de chaque radiateur. Son rôle est de réceptionner et décoder la trame reçue en liaison RF. Il se charge ensuite de retranscrire l’ordre sur l’interface fil pilote local à destination du radiateur sur lequel il est raccordé.

Le protocole Home Easy utilisé ici est exactement le même que celui des produits D-IO par Chacon.

### Prérequis:

Ce projet utilise les outils suivants:
 * [Microchip MPLAB X][2] (v2.15 ou plus récent)
 * [Microchip XC8 Compiler][3] (v1.32 ou plus récent)
 
[1]: http://www.microchip.com/wwwproducts/Devices.aspx?dDocName=en546902 "PIC 16F1825"
[2]: http://www.microchip.com/pagehandler/en-us/family/mplabx/ "MPLAB X"
[3]: http://www.microchip.com/pagehandler/en_us/devtools/mplabxc/ "MPLAB XC Compilers"
