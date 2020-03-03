
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <limits.h>

#include <iostream>

#define PORT 8080
#define V 9

using namespace std;

// ----- VARIABLES GLOBALES ----- //

int server_fd, new_socket;
struct sockaddr_in address;
int opt = 1;
int addrlen = sizeof(address);
int opcion;
int *graph;
int path[V];



// Programa que hace de servidor para calcular la ruta mas corta de un grafo generado aleatoriamente
// El servidor maneja la logica y el programa cliente recibe el grafo creado

/*!
 *  \brief
 *   La funcion CrearGrafo() aloca la memoria necesaria para el grafo y crea un arreglo auxiliar para mostrarlo al cliente,
 *   despues toma cada elemento del grafo (vertices, aristas y pesos; representados mediante una matriz de adyacencia) y genera un valor aleatorio dentro de un rango definido
 *   por ultimo, llena el arreglo auxiliar con los valores aleatorios generados en la matriz y la envia hacia el cliente
 *
 * @return
 */

int CrearGrafo()
{
    graph = (int*)malloc(sizeof(int*) * V * V);
    int arr[V][V];
    int FILA, COLUMNA;

    for (FILA = 0; FILA < V; FILA++)
        for (COLUMNA = 0; COLUMNA < V; COLUMNA++) {
            *(graph + FILA * V + COLUMNA) = rand() % 10;
            arr[FILA][COLUMNA] = *(graph+ FILA * V + COLUMNA);
        }
    send(new_socket, &arr, sizeof(int) * 9 * 9 , 0);

    return 0;
}

/*!
 *
 * \brief
 *  Funcion para encontrar el vertice con la medida de peso mas pequena.
 *  Se toman los vertices que no han sido valorados anteriormente.
 *
 * @param dist arreglo que contiene los valores de distancia del los nodos ya valorados.
 * @param sptSet significa "shortest path tree", un booleano que revisa si el nodo evaluado es parte de la ruta mas corta.
 * @return
 */

int minDistance(const int dist[], const bool sptSet[])
{

    int min = INT_MAX, min_index = 0;

    for (int v = 0; v < V; v++)
        if (!sptSet[v] && dist[v] <= min)
            min = dist[v], min_index = v;

    return min_index;
}


/*!
 * \brief
 *  Funcion que toma los valores de las distancias calculadas en la funcion anterior y las pasa a un arreglo auxiliar
 *  para luego enviarlo al cliente y que el cliente lo visualice
 *
 * @param dist arreglo con las distancias desde el nodo 0 a cada nodo del grafo
 * @return
 */

int generarPath(int *dist)
{
    for (int i = 0; i < V; i++) {
        path[i] = dist[i];
    }
    return 0;
}

/*!
 *
 *  \brief
 *  Funcion que toma el grafo (en forma de matriz de adyacencia) para calcular las distancias entre los nodos
 *  utilizando el algoritmo de dijkstra
 *
 *
 * @param graph matriz de adyacencia generada en la funcion CrearGrafo
 * @param src nombre del nodo con respecto al que se van a calcular todas las rutas
 * @return
 */


void *dijkstra(int *graph, int src)
{
    int dist[V]; // dist[i] tendra la distancia mas corta entre el nodo src y el nodo i

    bool sptSet[V]; // sptSet[i] sera True si el vertice i esta incluido en el shortest path tree o si la distancia de

    // Todas las distancias tienen inicialmente un valor de infinito
    for (int i = 0; i < V; i++)
        dist[i] = INT_MAX, sptSet[i] = false;

    // Distancia desde el vertice src hasta si mismo siempre es 0
    dist[src] = 0;

    // Encontrar el camino mas corto para todos los vertices
    for (int count = 0; count < V - 1; count++) {

        //Tomar las distancias minimas de los vertices no evaluados
        // u siempre es igual a src en el primer caso
        int u = minDistance(dist, sptSet);


        // Marcar el nodo procesado
        sptSet[u] = true;

        // Actualizar los valores de distancia de los vertices adyacentes al vertice evaluado
        for (int v = 0; v < V; v++)

            // Actualiza dist[v] solo si no esta en sptSet, si hay una arista desde
            // u hasta v, y si el peso total desde src a v es menor que el valor actual de dist[v]

            if (!sptSet[v] && *(graph+ u * V + v) && dist[u] != INT_MAX
                && dist[u] + *(graph+ u * V + v) < dist[v])
                dist[v] = dist[u] + *(graph+ u * V + v);
    }
    free(graph);

    // Enviar al cliente el arreglo de distancias

    generarPath(dist);
    send(new_socket, &path, sizeof(int) * V, 0);
}

/*!
 * \brief
 *  Funcion que crea el socket por el que el cliente se conectara al servidor
 *  se le asigna el puerto 8080
 *
 * @return
 */

int CrearSocket(){

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );

    if (bind(server_fd, (struct sockaddr *)&address,
             sizeof(address))<0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                             (socklen_t*)&addrlen))<0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }
}

/*!
 * \brief
 * Interfaz principal del servidor
 * El servidor maneja toda la logica de generar grafos y calcular sus distancias
 * El servidor no imprime nada directamente, los resultados de las funciones son visualizadas solo por el cliente
 *
 *
 * @param argc
 * @param argv
 * @return
 */

int main(int argc, char const *argv[])
{
    cout << "Esperando al cliente..." << endl;
    CrearSocket();

    bool exit = true;
    while(exit){
        recv(new_socket, &opcion, sizeof(int), 0);
        switch(opcion){
            case 1:
                cout << "El cliente ha seleccionado la opcion 1..." << endl;
                cout << "Se creara el grafo..." << endl;
                cout << "El grafo se ha creado y se ha enviado al cliente" << endl;
                CrearGrafo();
                cout << "-----------------------------------" << endl;
                opcion = 0;
                break;
            case 2:
                if (graph != nullptr){
                    cout << "El cliente ha seleccionado la opcion 2..." << endl;
                    cout << "Se calcularan distancias en el grafo..." << endl;
                    dijkstra(graph, 0);
                    cout << "Las distancias se han calculado y se han enviado al cliente" << endl;
                    cout << "-----------------------------------" << endl;
                    opcion = 0;
                } else{
                    cout << "Aun no se ha calculado el grafo..."  << endl;
                }
                break;

            case 3:
                cout << "El cliente ha elegido salir..." << endl;
                cout << "Desconectando..." << endl;
                exit = false;
                break;

            default:
                cout << "Opcion no valida" << endl;
                opcion = 0;
                cout << "-----------------------------------" << endl;
                break;
        }
    }
    return 0;
} 