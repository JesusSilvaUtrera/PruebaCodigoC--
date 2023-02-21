#include <iostream>
#include <string>
using namespace std;

//Ejemplo 1: structs anidados
struct fechaNacimiento
{
    int  dia;
    int  mes;
    int anyo;
};
 
struct datosPersona
{
    string nombre;
    char  inicial;
    struct fechaNacimiento diaDeNacimiento;
    float nota;
};
 
int main()
{
    datosPersona persona;
 
    persona.nombre = "Ignacio";
    persona.inicial = 'I';
    persona.diaDeNacimiento.mes = 8;
    persona.nota = 7.5;
    cout << "La nota es " << persona.nota;
 
    return 0;
}

//Ejemplo 2: structs junto con arrays
#include <iostream>
#include <string>
using namespace std;
 
int main()
{
    struct datosPersona
    {
        string nombre;
        char  inicial;
        int   edad;
        float nota;
    };
 
    //tipoStruct *nombre_array = new tipoStruct[tamaño]
    datosPersona *persona = new datosPersona[50];
 
    for (int i=0; i<5; i++)
    {
        cout << "Dime el nombre de la persona " << i << endl;
        cin >> persona[i].nombre;
    }
 
    cout << "La persona 3 es " << persona[2].nombre << endl;
 
    return 0;
}