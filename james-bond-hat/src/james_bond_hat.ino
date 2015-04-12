// ################################################################################
// #                    Projeto EuSonar - Modulo Bone                             #
// #                    Autor: Andre Perotti Netto                                #
// #                    Data criacao: 04/11/2015                                  #
// ################################################################################
// Prototipo do bone que visa evitar obstaculos

#include<NewPing.h>
// Constantes de configuracao geral
#define DelayEntrePing 50
#define DistanciaMediaBraco 50
#define DistanciaMediaPasso 90
#define Desligado 0

// Constantes que representam os motores de vibracao
#define MotorFrontal  5
#define MotorEsquerdo 6
#define MotorDireito  7
#define MotorTrazeiro 8

// Constantes que representam os leds que ajudam a debugar
#define LedFrontal    9
#define LedEsquerdo   10
#define LedDireito    11
#define LedTrazeiro   12

// Configuracao da distancia maxima dos sensores
#define DistanciaMaximaUltrasom

// Constantes do sensor ultrasonico frontal
#define UltrasonicoFrontalTrigger     13
#define UltrasonicoFrontalEcho        14

// Constantes do sensor ultrasonico da lateral esquerda
#define UltrasonicoEsquerdoTrigger    15
#define UltrasonicoEsquerdoEcho       16

// Constantes do sensor ultrasonico da lateral direita
#define UltrasonicoDireitoTrigger     17
#define UltrasonicoDireitoEcho        18

// Constantes do sensor ultrasonico da lateral direita
#define UltrasonicoDireitoTrigger     19
#define UltrasonicoDireitoEcho        20

// Estrutura que controla em qual estado do programa estamos rodando
enum EstadoDeExecucao
{
    EstadoInicial=1,
    NavegacaoComMotores,
    NavegacaoComMotoresLed,
    NavegacaoComMotoresLedSom,
    ControleRemotoComMotores,
    ControleRemotoComMotoresSom,
    ControleRemotoComMotoresSomLed, 
    ReplayDeRotaSemAvisoDeObstaculo,
    ReplayDeRotaComAvisoDeObstaculoVibracao,
    ReplayDeRotaComAvisoDeObstaculoVibracaoSom,
    ReplayDeRotaComAvisoDeObstaculoVibracaoSomLed,
    GravacaoRota, 
    TesteDebug
};

typedef enum EstadoDeExecucao estado;

// inicia objetos que implemetam o sensores ultrasonico
NewPing UltrasomFrontal(
    UltrasonicoFrontalTrigger, 
    UltrasonicoFrontalEcho,
    DistanciaMaximaUltrasonico
);

NewPing UltrasomLateralEsquerdo(
    UltrasonicoEsquerdoTrigger, 
    UltrasonicoEsquerdoEcho,
    DistanciaMaximaUltrasonico
);

NewPing UltrasomLateralDireito(
    UltrasonicoDireitoTrigger, 
    UltrasonicoDireitoEcho,
    DistanciaMaximaUltrasonico
);

//  somente valores positivos e por isso serao unsigned
unsigned int DistanciaFrontal;
unsigned int DistanciaFrontalAnterior;
unsigned int DistanciaLateralEsquerda;
unsigned int DistanciaLateralEsquerdaAnterior;
unsigned int DistanciaLateralDireita;
unsigned int DistanciaLateralDireitaAnterior;
unsigned int Duracao;
unsigned int tempo;
//  declara o estado inicial que representa o que o app
//  esta fazendo no momento
estado estado_atual_app; 


// Retorna a diferenca do valor atual com valor anterior.
int compara_distancias(unsigned int valor_atual, unsigned int valor_anterior);
// Aciona ou para de acinar os motores de acordo com a comparacao das distancias.
void trata_retorno_das_distancia_com_vibracao(int diferenca_das_distancias, String intencidade);
void setup()
{
    DistanciaFrontal = 0;
    DistanciaFrontalAnterior = 0;
    DistanciaLateralEsquerda = 0;
    DistanciaLateralEsquerdaAnterior = 0;
    DistanciaLateralDireita = 0;
    DistanciaLateralDireitaAnterior = 0;

    estado_atual_app = EstadoInicial;
    // Descomente a linha de baixo para debugar
    // Serial.begin(115200);

    // Inicia pinos dos motores como saida
    pinMode(MotorFrontal, OUTPUT);
    pinMode(MotorEsquerdo, OUTPUT);
    pinMode(MotorDireito, OUTPUT);
    pinMode(MotorTrazeiro, OUTPUT);

    // Inicia pinos dos led como saida
    pinMode(LedFrontal, OUTPUT);
    pinMode(LedEsquerdo, OUTPUT);
    pinMode(LedDireito, OUTPUT);
    pinMode(LedTrazeiro, OUTPUT);
}

void loop()
{

    // Independentemente do estado, sempre leio o ultrasom.
    scan();

    // verifico em qual estado de execucao esta o app e chamo 
    // sua respectiva funcao

    switch( estado_atual_app )
    {

        case EstadoIniciali :
            break;
        case NavegacaoComMotores :
            break;
        case NavegacaoComMotoresLed :
            break;
        case NavegacaoComMotoresLedSom :
            break;
        case ControleRemotoComMotores :
            break;
        case ControleRemotoComMotoresSom :
            break;
        case ControleRemotoComMotoresSomLed :
            break;
        case ReplayDeRotaSemAvisoDeObstaculo :
            break;
        case ReplayDeRotaComAvisoDeObstaculoVibracao :
            break;
        case ReplayDeRotaComAvisoDeObstaculoVibracaoSom :
            break;
        case ReplayDeRotaComAvisoDeObstaculoVibracaoSomLed :
            break;
        case GravacaoRota :
            break;
        case TesteDebug :
            break;
        default:

    }

}

// Faz a leitura de todos os sensores ultrasom e seta as variaveis com as distancias lidas.
void scan()
{

    DistanciaFrontalAnterior = DistanciaFrontal;
    DistanciaFrontal = Tempo = 0;
    delay(DelayEntrePing); 
    Tempo = UlstrasomFrontal.ping(); 
    DistanciaFrontal = Tempo / US_ROUNDTRIP_CM;

    DistanciaLateralEsquerdaAnterior = DistanciaFrontal;
    DistanciaLateralEsquerda = Tempo = 0;
    delay(DelayEntrePing); 
    Tempo = UlstrasomLateralEsquerdo.ping(); 
    DistanciaLateralEsquerda = Tempo / US_ROUNDTRIP_CM;

    DistanciaLateralDireitaAnterior = DistanciaLateralDireita;
    DistanciaLateralDireita = Tempo = 0;
    delay(DelayEntrePing); 
    Tempo = UlstrasomLateralDireito.ping(); 
    DistanciaLateralDireita = Tempo / US_ROUNDTRIP_CM;

}

// De acordo com as distancias lidas por scan() vibra os motores.
void navegacao_com_motor()
{

    // TODO: chamar as funcoes que tratam as distancias e vibrem os motores
    tratamento_motor("Frontal");
    tratamento_motor("Esquerdo");
    tratamento_motor("Direito");

}

void tratamento_motor(String posicao)
{
    int distancia_medida = 0; // vai receber a distancia de um dos sensores
    // Verifa se a distancia aumentou ou diminuio.
    int comparacao_distancia = compara_distancias(DistanciaFrontal, DistanciaFrontalAnterior);
    if( posicao == "Frontal" )
    {
        distancia_medida = DistanciaFrontal;
    }
    else if ( posicao == "Esquerdo" );
    {
        distancia_medida = DistanciaLateralEsquerda;
    }
    else if ( posicao == "Direito" )
    {
        distancia_medida = DistanciaLateralDireita;
    }

    if( distancia_medida > DistanciaMediaPasso ||  == Desligado )
    {
        // Distancia segura, desliga motores de vibracao.
        // TODO: fazer funcao que desligue os motores.
        desliga_motor();
    }
    // Caso exista um objeto entre 50 e 90 cm de um aviso vibratorio medio.
    else if ( (distancia_medida > DistanciaMediaBraco || distancia_medida == Desligado) &&
              (distancia_medida < DistanciaMediaPasso || distancia_medida == Desligado) )
    {
        // TODO: chamar funcao que vibra com potencia media
        // Compara os valores para saber se o objeto esta se aproximando e da um alerta medio
        trata_retorno_das_distancia_com_vibracao(comparacao_distancia, "MEDIA");
    }
    // Caso em que o objeto se encontra em uma distancia menor que 50 cm, alerta com vibracao forte.
    else
    {
        // TODO: chamar funcao que vibra com potencia forte
        trata_retorno_das_distancia_com_vibracao(comparacao_distancia, "FORTE");
    }
}

// Verifica distancias do ultrasom frontal e chama as funcoes de vibracao de acordo com a distancia.
void tratamento_motor_frontal()
{

    // Verifa se a distancia aumentou ou diminuio.
    int comparacao_distancia = compara_distancias(DistanciaFrontal, DistanciaFrontalAnterior);
    // Caso nao haja nenhum objeto muito proximo, isto e, mais de 90 cm de distancia fontal.
    if( DistanciaFrontal > DistanciaMediaPasso || DistanciaFrontal == Desligado )
    {

        // Distancia segura frontalmente, desliga motores de vibracao.
        // TODO: fazer funcao que desligue os motores.
        desliga_motor();

    }
    // Caso exista um objeto entre 50 e 90 cm de um aviso vibratorio medio.
    else if ( (DistanciaFrontal > DistanciaMediaBraco || DistanciaFrontal == Desligado) &&
              (DistanciaFrontal < DistanciaMediaPasso || DistanciaFrontal == Desligado) )
    {

        // TODO: chamar funcao que vibra com potencia media
        // Compara os valores para saber se o objeto esta se aproximando
        trata_retorno_das_distancia_com_vibracao(comparacao_distancia, "MEDIA");

    }
    // Caso em que o objeto se encontra em uma distancia menor que 50 cm, alerta com vibracao forte.
    else
    {

        // TODO: chamar funcao que vibra com potencia forte
        trata_retorno_das_distancia_com_vibracao(comparacao_distancia, "FORTE");

    }

}

// Verifica distancias do ultrasom esquerdo e chama as funcoes de vibracao de acordo com a distancia.
void tratamento_motor_esquerdo()
{

    // Verifa se a distancia aumentou ou diminuio.
    int comparacao_distancia = compara_distancias(DistanciaLateralEsquerda, DistanciaLateralEsquerdaAnterior);
    // Caso nao haja nenhum objeto muito proximo, isto e, mais de 90 cm de distancia lateral.
    if( DistanciaLateralEsquerda > DistanciaMediaPasso || DistanciaLateralEsquerda == Desligado )
    {

        // Distancia segura lateral esquerda, desliga motores de vibracao.
        // TODO: fazer funcao que desligue os motores.
        desliga_motor();

    }
    // Caso exista um objeto entre 50 e 90 cm de um aviso vibratorio medio.
    else if ( (DistanciaLateralEsquerda > DistanciaMediaBraco || DistanciaLateralEsquerda == Desligado) &&
              (DistanciaLateralEsquerda < DistanciaMediaPasso || DistanciaLateralEsquerda == Desligado) )
    {

        // TODO: chamar funcao que vibra com potencia media
        // Compara os valores para saber se o objeto esta se aproximando
        trata_retorno_das_distancia_com_vibracao(comparacao_distancia, "MEDIA");

    }
    // Caso em que o objeto se encontra em uma distancia menor que 50 cm, alerta com vibracao forte.
    else
    {

        // TODO: chamar funcao que vibra com potencia forte
        trata_retorno_das_distancia_com_vibracao(comparacao_distancia, "FORTE");

    }

}
// Retorna a diferenca do valor atual com valor anterior
int compara_distancias(unsigned int valor_atual, unsigned int valor_anterior)
{

    return valor_atual - valor_anterior; 

}
// Se a distancia esta diminuindo continua vibrando se a distancia se estabeliza ou aumenta, para.
void trata_retorno_das_distancia_com_vibracao(int diferenca_das_distancias, String intencidade)
{

        if( diferenca_das_distancias < 0 )
        {

            // Distancia esta diminuido, aumenta vibracao ou continua o aviso
            // TODO: fazer funcao que aumenta vibracao
            if( intencidade == "MEDIA")
            {

                //TODO: Fazer funcao que liga o motor com intensidade media.
                liga_motor("MEDIA"); // Fazer esta funcao.

            }
            else if ( intencidade == "FORTE")
            {

                //TODO: Fazer funcao que liga o motor com intensidade forte.
                liga_motor("FORTE"); // Fazer esta funcao.

            }

        }
        else
        {

            // Distancia se manteve ou esta aumento, diminui vibracao ou pare de vibrar
            // TODO: Fazer funcao que diminui vibracao ou deliga motor, por enquanto desligamos.
            desliga_motor(); // Fazer esta funcao.

        }

}
