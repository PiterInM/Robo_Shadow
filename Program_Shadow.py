# Importação de bibliotecas OpenCV, MediaPipe, Math e PySerial
import cv2
import mediapipe as mp
from math import degrees, sqrt, atan
import serial

# Declaração porta do ESP32
porta = '/dev/ttyUSB0'  # Coloque entre as aspas simples a posta serial do Esp32
velocComunc = 115200  # Coloque aqui a velocidade da comunicação serial

# Definindo cores
corPontos = (0, 0, 255)
corTexto = (0, 0, 0)
amarelo = (200, 200, 0)
magenta = (200, 0, 200)
ciano = (0, 200, 200)

# Loop de menu, perguntar o usuário de deseja conectar ao Esp32
while True:
    conecEsp = str(input('Deseja conectar com esp?\n'
                         '\033[34m[ 1 ]\033[m - \033[34mSIM\033[m\n'
                         '\033[34m[ 2 ]\033[m - \033[34mNÂO\033[m\n'
                         '-> '))
    if conecEsp in '1 2':
        break
    else:
        print('\033[35mINVÁLIDO!\033[m Insira 1 ou 2\n')

# Sendo a resposta do loop SIM, tentar fazer conecção serial
if conecEsp == '1':
    while True:
        try:
            esp = serial.Serial(porta, velocComunc)
            print('Esp Conectado')
            break
        except:
            pass

# Declarações de vídeo
video = cv2.VideoCapture(0)  # Altere esse valor para trocar de câmera
pose = mp.solutions.pose
Pose = pose.Pose(min_tracking_confidence=0.75,
                 min_detection_confidence=0.75)  # Altere esses valores para definir a minima precisão para detecção
draw = mp.solutions.drawing_utils

# Loop de execução (Reconhecimento e comunicação)
while True:
    conectado, vid = video.read()  # Coloca em "vid" as imagens recebidas pela câmera
    vid = cv2.resize(vid, (0, 0), fx=1.7, fy=1.7)  # Zoom em vid
    results = Pose.process(vid)  # Processamento de Vid
    points = results.pose_landmarks  # Calcula os pontos
    # draw.draw_landmarks(vid, points, pose.POSE_CONNECTIONS)  # Retire o comentário dessa linha mostrar todos os pontos

    h, w, _ = vid.shape  # Armazena altura e largura do vídeo

    # Tendo encontrado uma pessoa, armazena informações
    if points:
        nariz = points.landmark[pose.PoseLandmark.NOSE]  # Armazena coordenadas do ponto do nariz
        cv2.circle(vid, (int(nariz.x * w), int(nariz.y * h)), 2, corPontos, 2)  # Ponto na imagem

        ombroE = points.landmark[pose.PoseLandmark.LEFT_SHOULDER]  # Armazena coordenadas do ponto do ombro esquerdo
        cv2.circle(vid, (int(ombroE.x * w), int(ombroE.y * h)), 2, corPontos, 2)  # Ponto na imagem

        ombroD = points.landmark[pose.PoseLandmark.RIGHT_SHOULDER]  # Armazena coordenadas do ponto do ombro direito
        cv2.circle(vid, (int(ombroD.x * w), int(ombroD.y * h)), 2, corPontos, 2)  # Ponto na imagem

        quadrilE = points.landmark[pose.PoseLandmark.LEFT_HIP]  # Armazena coordenadas do ponto do quadril a esquerda
        cv2.circle(vid, (int(quadrilE.x * w), int(quadrilE.y * h)), 2, corPontos, 2)  # Ponto na imagem

        quadrilD = points.landmark[pose.PoseLandmark.RIGHT_HIP]  # Armazena coordenadas do ponto do quadril a direita
        cv2.circle(vid, (int(quadrilD.x * w), int(quadrilD.y * h)), 2, corPontos, 2)  # Ponto na imagem

        cotoveloE = points.landmark[pose.PoseLandmark.LEFT_ELBOW]  # Armazena coordenadas do ponto do cotovelo esquerdo
        cv2.circle(vid, (int(cotoveloE.x * w), int(cotoveloE.y * h)), 2, corPontos, 2)  # Ponto na imagem

        cotoveloD = points.landmark[pose.PoseLandmark.RIGHT_ELBOW]  # Armazena coordenadas do ponto do cotovelo direito
        cv2.circle(vid, (int(cotoveloD.x * w), int(cotoveloD.y * h)), 2, corPontos, 2)  # Ponto na imagem

        joelhoE = points.landmark[pose.PoseLandmark.LEFT_KNEE]  # Armazena coordenadas do ponto do joelho esquerdo
        cv2.circle(vid, (int(joelhoE.x * w), int(joelhoE.y * h)), 2, corPontos, 2)  # Ponto na imagem

        joelhoD = points.landmark[pose.PoseLandmark.RIGHT_KNEE]  # Armazena coordenadas do ponto do joelho direito
        cv2.circle(vid, (int(joelhoD.x * w), int(joelhoD.y * h)), 2, corPontos, 2)  # Ponto na imagem

        calcanharE = points.landmark[pose.PoseLandmark.LEFT_HEEL]  # Armazena coordenadas do ponto do calcanhar esquerdo
        cv2.circle(vid, (int(calcanharE.x * w), int(calcanharE.y * h)), 2, corPontos, 2)  # Ponto na imagem

        calcanharD = points.landmark[pose.PoseLandmark.RIGHT_HEEL]  # Armazena coordenadas do ponto do calcanhar direito
        cv2.circle(vid, (int(calcanharD.x * w), int(calcanharD.y * h)), 2, corPontos, 2)  # Ponto na imagem

        maoE = points.landmark[pose.PoseLandmark.LEFT_WRIST]  # Armazena coordenadas do ponto do punho esquerdo
        cv2.circle(vid, (int(maoE.x * w), int(maoE.y * h)), 2, corPontos, 2)  # Ponto na imagem

        maoD = points.landmark[pose.PoseLandmark.RIGHT_WRIST]  # Armazena coordenadas do ponto do punho direito
        cv2.circle(vid, (int(maoD.x * w), int(maoD.y * h)), 2, corPontos, 2)  # Ponto na imagem

        peE = points.landmark[pose.PoseLandmark.LEFT_FOOT_INDEX]  # Armazena coordenadas do ponto do pé esquerdo
        cv2.circle(vid, (int(peE.x * w), int(peE.y * h)), 2, corPontos, 2)  # Ponto na imagem

        peD = points.landmark[pose.PoseLandmark.RIGHT_FOOT_INDEX]  # Armazena coordenadas do ponto do pé direito
        cv2.circle(vid, (int(peD.x * w), int(peD.y * h)), 2, corPontos, 2)  # Ponto na imagem

        # Desenhos de teste
        """
        cv2.line(vid, (int(quadrilE.x * w), int(quadrilE.y * h)), (int(joelhoE.x * w), int(joelhoE.y * h)), amarelo, 2)
        cv2.line(vid, (int(quadrilE.x * w), int(quadrilE.y * h)), (int(quadrilE.x * w), int(joelhoE.y * h)), ciano, 2)
        cv2.line(vid, (int(joelhoE.x * w), int(joelhoE.y * h)), (int(quadrilE.x * w), int(joelhoE.y * h)), magenta, 2)

        cv2.line(vid, (int(ombroD.x * w), int(ombroD.y * h)), (int(ombroD.x * w), int(cotoveloD.y * h)), amarelo,2)
        cv2.line(vid, (int(cotoveloD.x * w), int(cotoveloD.y * h)), (int(ombroD.x * w), int(cotoveloD.y * h)), ciano, 2)
        cv2.line(vid, (int(cotoveloD.x * w), int(cotoveloD.y * h)), (int(ombroD.x * w), int(ombroD.y * h)), magenta, 2)
        """
        # Calculos

        # Ângulo lateral braço esquerdo
        # Pelas coordenadas tem-se um triângulo retângulo de forma que seus vértices são os pares de coordenadas:
        # 1. X e Y do ponto no Ombro Esquerdo;
        # 2. X e Y do ponto no Cotovelo Esquerdo;
        # 3. X do ponto no Ombro Esquerdo e Y do ponto no Cotovelo Esquerdo.
        #
        # Ligando estas arestas tem-se os vértices do triângulo retângulo, tal que:
        # CO = Comprimento do cateto oposto ao ângulo desejado (ombroE.x - cotoveloE.x)
        # CA = Comprimento do cateto adjacente ao ângulo desejado (ombroE.y - cotoveloE.y)
        #
        # A partir desses segmentos, podemos obter a tangente do ângulo desejado por CO / CA.
        # Utilizamos a função de arco tangente para obter o ângulo em radiano e por último a
        # função que converte esse ângulo em graus.
        angBe = int(  # Inteiro
            degrees(  # Converte radiano em graus
                atan(  # Arco tangente (arc tg) (transformar a tg em ângulo [rad])
                    (ombroE.x - cotoveloE.x) / (ombroE.y - cotoveloE.y))))  # Cateto oposto / adjacente (tg)
        # Limitadores
        if cotoveloE.x < ombroE.x and cotoveloE.y > ombroE.y:
            angBe = 0
        elif cotoveloE.x < ombroE.x and cotoveloE.y < ombroE.y:
            angBe = 180
        elif angBe < 0:
            angBe = 180 + angBe

        # Ângulo lateral braço direito
        # De forma análoga ao mesmo ângulo no braço esquerdo, pelas coordenadas dos pontos:
        # 1. X e Y do ponto no Ombro Direito;
        # 2. X e Y do ponto no Cotovelo Direito;
        # 3. X do ponto no Ombro Direito e Y do ponto no Cotovelo Direito.
        #
        # Ligando estas arestas tem-se os vértices do triângulo retângulo, tal que:
        # CO = Comprimento do cateto oposto ao ângulo desejado (ombroD.x - cotoveloD.x)
        # CA = Comprimento do cateto adjacente ao ângulo desejado (ombroD.y - cotoveloD.y)
        #
        # A partir desses segmentos, podemos obter a tangente do ângulo desejado por CO / CA.
        # Utilizamos a função de arco tangente para obter o ângulo em radiano e por último a
        # função que converte esse ângulo em graus.
        angBd = int(  # Inteiro
            degrees(  # Converte radiano em graus
                atan(  # Arco tangente (arc tg) (transformar a tg em ângulo [rad])
                    (ombroD.x - cotoveloD.x) / (ombroD.y - cotoveloD.y))))  # Cateto oposto / adjacente (tg)
        # Limitadores
        if cotoveloD.x > ombroD.x and cotoveloD.y > ombroD.y:
            angBd = 180
        elif cotoveloD.x > ombroD.x and cotoveloD.y < ombroD.y:
            angBd = 0
        elif angBd < 0:
            angBd = 180 + angBd

        # Ângulo lateral cabeça
        # O ângulo da cabeça vem da proporção da distância δ em relação a distância dos ombros, em que
        # δ / ED = angC / 180, tal que:
        # δ = Distância entre o ombro esquerdo e o segmento da altura
        # ED = Distância entre os ombros (sqrt(((ombroD.x - ombroE.x) ** 2) + ((ombroD.y - ombroE.y) ** 2)))
        # angC = Ângulo desejado
        #
        # Proporção δ entre nariz e ombro -> δ = (ED² + EN² - DN²) / ED, tal que:
        # δ = Distância entre o ombro esquerdo e o segmento da altura
        # ED = Distância entre os ombros (sqrt(((ombroD.x - ombroE.x) ** 2) + ((ombroD.y - ombroE.y) ** 2)))
        # EN = Distância entre ombro esquerdo e nariz (sqrt(((ombroE.x - nariz.x) ** 2) + ((ombroE.y - nariz.y) ** 2)))
        # DN = Distância entre ombro direito e nariz (sqrt(((ombroD.x - nariz.x) ** 2) + ((ombroD.y - nariz.y) ** 2)))
        #
        # Dessa forma angC = ((ED² + EN² - DN²) * 180) / 2ED =
        # ((((sqrt(((ombroD.x - ombroE.x) ** 2) + ((ombroD.y - ombroE.y) ** 2))) ** 2) +
        # ((sqrt(((ombroE.x - nariz.x) ** 2) + ((ombroE.y - nariz.y) ** 2))) ** 2) -
        # ((sqrt(((ombroD.x - nariz.x) ** 2) + ((ombroD.y - nariz.y) ** 2))) ** 2)) * 180) /
        # (2 * ((sqrt(((ombroD.x - ombroE.x) ** 2) + ((ombroD.y - ombroE.y) ** 2))) ** 2) =
        #
        # (((ombroD.x - ombroE.x) ** 2 + (ombroD.y - ombroE.y) ** 2) +
        # ((ombroE.x - nariz.x) ** 2 + (ombroE.y - nariz.y) ** 2) -
        # ((ombroD.x - nariz.x) ** 2 + (ombroD.y - nariz.y) ** 2)) * 180 /
        # (2 * ((ombroD.x - ombroE.x) ** 2 + (ombroD.y - ombroE.y) ** 2))
        #
        # Além disso, fez-se necessária a inversão do ângulo, que ficou 180 - valor calculado
        auxCab = (((((ombroD.x - ombroE.x) ** 2) + ((ombroD.y - ombroE.y) ** 2)) +  # ED² +
                   (((ombroE.x - nariz.x) ** 2) + ((ombroE.y - nariz.y) ** 2)) -  # EN² -
                   (((ombroD.x - nariz.x) ** 2) + ((ombroD.y - nariz.y) ** 2))) /  # DN² /
                  (2 * sqrt(((ombroD.x - ombroE.x) ** 2) + ((ombroD.y - ombroE.y) ** 2))))  # 2ED
        angC = (180 -  # Inversão do ângulo
                int(auxCab * 180 / (  # ((ED² + EN² - DN²) * 180) /
                    sqrt(((ombroD.x - ombroE.x) ** 2) + ((ombroD.y - ombroE.y) ** 2)))))  # ED

        # Limitadores
        if angC > 180:
            angC = 180
        elif angC < 0:
            angC = 0

        # Alturas tronco por Teorema de Pitágoras
        comObrQuadE = sqrt((quadrilE.x - ombroE.x) ** 2 + (quadrilE.y - ombroE.y) ** 2)
        comObrQuadD = sqrt((quadrilD.x - ombroD.x) ** 2 + (quadrilD.y - ombroD.y) ** 2)

        # Calcula o comprimento do cotovelo ao ombro esquerdo
        # Comprimento dado pelo Teorema de Pitágoras utilizando dos pontos do cotovelo e ombro esquerdo
        comBracoE = sqrt(((cotoveloE.y - ombroE.y) ** 2) + ((cotoveloE.x - ombroE.x) ** 2))
        # Ângulo frontal braço esquerdo
        # Ângulo dado pela relação comBracoE / comObrQuadE = angBraE / 180
        angBraE = int(  # Inteiro
            comBracoE * 180 / comObrQuadE)  # Proporção comprimento ângulo

        # Calcula o comprimento do cotovelo ao ombro direito
        # Comprimento dado pelo Teorema de Pitágoras utilizando dos pontos do cotovelo e ombro direito
        comBracoD = sqrt(((cotoveloD.y - ombroD.y) ** 2) + ((cotoveloD.x - ombroD.x) ** 2))
        # Ângulo frontal braço direito
        # Ângulo dado pela relação comBracoD / comObrQuadD = angBraD / 180
        # Sendo necessária a inversão do ângulo é retirado de 180 o valor calculado
        angBraD = int(  # Inteiro
            180 -  # Inversão do ângulo
            comBracoD * 180 / comObrQuadD)  # Proporção comprimento ângulo

        # Calcula o comprimento do cotovelo a mão esquerda
        # Comprimento dado pelo Teorema de Pitágoras utilizando dos pontos do cotovelo e do pulso esquerdo
        comAntBracoE = sqrt(((maoE.y - cotoveloE.y) ** 2) + ((maoE.x - cotoveloE.x) ** 2))
        # Ângulo cotovelo esquerdo
        # Ângulo dado pela relação comAntBracoE / comObrQuadE = angCotE / 180
        # Sendo necessária a inversão do ângulo é retirado de 180 o valor calculado
        # Na tentativa de compensar o ângulo frontal do braço, o valor calculado soma-se a 90 - ângulo frontal do braço
        angCotE = (90 - angBraE +  # Compensação
                   int(  # Inteiro
                       180 -  # Inversão do ângulo
                       comAntBracoE * 180 / comObrQuadE))  # Proporção comprimento ângulo

        # Calcula o comprimento do cotovelo a mão direita
        # Comprimento dado pelo Teorema de Pitágoras utilizando dos pontos do cotovelo e do pulso direito
        comAntBracoD = sqrt(((maoD.y - cotoveloD.y) ** 2) + ((maoD.x - cotoveloD.x) ** 2))
        # Ângulo cotovelo direito
        # Ângulo dado pela relação comAntBracoD / comObrQuadD = angCotD / 180
        # Na tentativa de compensar o ângulo frontal do braço, o valor calculado soma-se a 90 - ângulo frontal do braço
        angCotD = (90 - angBraD +  # Compensação
                   int(  # Inteiro
                       comAntBracoD * 180 / comObrQuadD))  # Proporção comprimento ângulo

        # Ângulo lateral perna direita
        # De forma análoga ao ângulo lateral nos braços:
        # Podemos obter a tangente do ângulo desejado, pois tem-se os catetos do triângulo retângulo.
        # Utilizamos a função de arco tangente para obter o ângulo em radiano e por último a
        # função que converte esse ângulo em graus.
        # Como o movimento das pernas tem amplitude de 90º, usa somente 90 + o ângulo calculado
        angPe = 90 + int(  # Inteiro
            degrees(  # Converte radiano em graus
                atan(  # Arco tangente (arc tg) (transformar a tg em ângulo [rad])
                    (quadrilE.x - joelhoE.x) / (quadrilE.y - joelhoE.y))))  # Cateto oposto / adjacente (tg)

        # Ângulo lateral perna esquerda
        # De forma análoga ao ângulo lateral nos braços e similar ao mesmo ângulo na outra perna:
        # Podemos obter a tangente do ângulo desejado, pois tem-se os catetos do triângulo retângulo.
        # Utilizamos a função de arco tangente para obter o ângulo em radiano e por último a
        # função que converte esse ângulo em graus.
        # Como o movimento das pernas tem amplitude de 90º, usa somente 90 + o ângulo calculado
        angPd = 90 + int(  # Inteiro
            degrees(  # Converte radiano em graus
                atan(  # Arco tangente (arc tg) (transformar a tg em ângulo [rad])
                    (quadrilD.x - joelhoD.x) / (quadrilD.y - joelhoD.y))))  # Cateto oposto / adjacente (tg)

        # Comprimento coxa Esquerda
        comCoxE = sqrt((joelhoE.y - quadrilE.y) ** 2 + (joelhoE.x - quadrilE.x) ** 2)
        # Ângulo frontal perna esquerda
        angCoxE = int(  # Inteiro
            comCoxE * 90 / (comObrQuadE / 5 * 4))

        # Comprimento coxa Direita
        comCoxD = sqrt((joelhoD.y - quadrilD.y) ** 2 + (joelhoD.x - quadrilD.x) ** 2)
        # Ângulo frontal perna direita
        angCoxD = (180 -  # Inversão do ângulo
                   int(  # Inteiro
                       comCoxD * 90 / (comObrQuadD / 5 * 4)))

        # Comprimento canela Esquerda
        comCanE = sqrt(((calcanharE.y - joelhoE.y) ** 2) + ((calcanharE.x - joelhoE.x) ** 2))
        # Ângulo joelho esquerdo
        angJe = (90 - angCoxE +  # Compensação
                 int(  # Inteiro
                     comCanE * 90 / (comObrQuadE / 5 * 4)))
        # Limitador
        if angJe > 90:
            angJe = 90

        # Comprimento canela Direita
        comCanD = sqrt(((calcanharD.y - joelhoD.y) ** 2) + ((calcanharD.x - joelhoD.x) ** 2))
        # Ângulo joelho direito
        angJd = (90 - angCoxD +  # Compensação
                 180 -  # Inversão do ângulo
                 int(  # Inteiro
                     comCanD * 90 / (comObrQuadD / 5 * 4)))
        # Limitador
        if angJd < 90:
            angJd = 90

        # Comunicação com esp
        if conecEsp == '1':
            esp.write(str(angBd).encode())
            esp.write('q'.encode())
            esp.write(str(angBe).encode())
            esp.write('w'.encode())
            esp.write(str(angC).encode())
            esp.write('e'.encode())
            esp.write(str(angBraE).encode())
            esp.write('r'.encode())
            esp.write(str(angBraD).encode())
            esp.write('t'.encode())
            esp.write(str(angCotE).encode())
            esp.write('y'.encode())
            esp.write(str(angCotD).encode())
            esp.write('u'.encode())
            esp.write(str(angPe).encode())
            esp.write('i'.encode())
            esp.write(str(angPd).encode())
            esp.write('o'.encode())
            esp.write(str(angCoxE).encode())
            esp.write('p'.encode())
            esp.write(str(angCoxD).encode())
            esp.write('a'.encode())
            esp.write(str(angJe).encode())
            esp.write('s'.encode())
            esp.write(str(angJd).encode())
            esp.write('d'.encode())
            esp.flush()

        # Escrever na tela
        """
        cv2.putText(vid, f'D: {angBd:.1f}', (2, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, corTexto)
        cv2.putText(vid, f'E: {angBraE:.1f}', (2, 105), cv2.FONT_HERSHEY_SIMPLEX, 1.5, corTexto)
        cv2.putText(vid, f'D: {angBraD:.1f}', (2, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.5, corTexto)
        cv2.putText(vid, f'E: {angCotE:.1f}', (2, 175), cv2.FONT_HERSHEY_SIMPLEX, 1.5, corTexto)
        cv2.putText(vid, f'D: {angCotD:.1f}', (2, 210), cv2.FONT_HERSHEY_SIMPLEX, 1.5, corTexto)"""

    cv2.imshow('video', vid)  # Abrir janela de vídeo
    vid = cv2.flip(vid, 1)  # Espelhar vídeo

    # 'q' Para sair do loop
    if cv2.waitKey(1) == ord('q'):
        break

# Limpeza de cache
video.release()
cv2.destroyAllWindows()