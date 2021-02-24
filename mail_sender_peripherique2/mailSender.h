#ifndef MAIL_SENDER_H_INCLUDED
#define MAIL_SENDER_H_INCLUDED


bool mail(String sujet,String destAddress,String message);


/*
211 État système, ou réponse d'aide système.
214 Message d'aide [Informations sur l'utilisation d'un récepteur ou signification d'une commande non standard particulière ; utile seulement pour un utilisateur humain].
220 <domaine> Service disponible.
221 <domaine> Canal de transmission en cours de fermeture.
250 Action de messagerie effectuée, succès.
251 Utilisateur non local ; réémission vers <route-directe> (avec relais automatique).
354 Début du corps du message ; arrêt par <CRLF>.<CRLF>.
421 <domaine> Service non disponible, canal en fermeture [Réponse à émettre sur tous les canaux lorsque le système exécute une séquence d'arrêt].
450 Action non effectuée : boîte aux lettres non disponible [Ex. : boîte aux lettres occupée].
451 Action arrêtée : erreur de traitement.
452 Action non effectuée : manque de ressources système.
500 Erreur de syntaxe, commande non reconnue [y compris des erreurs de type « ligne de commande trop longue »].
501 Erreur de syntaxe dans les paramètres ou arguments.
502 Commande non implémentée.
503 Mauvaise séquence de commandes.
504 Paramètre de commande non implémenté.
550 Action non effectuée : boîte aux lettres non disponible [Ex. : boîte aux lettres non trouvée, pas d'accès].
551 Utilisateur non local ; essayer <route-directe> (sans relais automatique)
552 Action annulée : manque de ressources de stockage.
553 Action non effectuée : nom de boîte aux lettres non autorisée [Ex. : erreur de syntaxe dans le nom de boîte].
554 Transaction échouée.
 */


#endif // MAIL_SENDER_H_INCLUDED
