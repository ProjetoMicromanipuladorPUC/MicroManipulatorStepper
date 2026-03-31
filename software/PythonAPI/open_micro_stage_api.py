import threading
import time
import re
from enum import Enum

import serial
import numpy as np
from colorama import Fore, Style, init

# ==================================================================================================
# GUIA DE APRENDIZAGEM (ROTEIRO PRATICO)
# PARTE 1: Protocolo base (status e niveis de log)
# PARTE 2: Inicializacao da serial e sincronizacao entre threads
# PARTE 3: Conexao/reconexao fisica da porta
# PARTE 4: Loop de leitura assincrona da serial
# PARTE 5: Classificacao de cada linha recebida
# PARTE 6: Envio bloqueante de comando e espera por resposta
# PARTE 7: API de alto nivel (metodos de uso do microestagio)
# PARTE 8: Ciclo de vida e callbacks visuais
# PARTE 9: Comandos de movimento e calibracao
# PARTE 10: Leitura de estado e parametros de controle
# PARTE 11: Parser utilitario de tabelas retornadas pelo firmware
# ==================================================================================================

# --- SerialInterface --------------------------------------------------------------------------------------------------
# PARTE 1: Camada de protocolo serial (baixo nivel).
# Camada de baixo nivel: abre a serial, envia comandos e interpreta respostas do firmware.
# Ela nao conhece conceitos do micromanipulador (home, move_to etc.); apenas protocolo de comunicacao.

class SerialInterface:
    # PARTE 1.1: Estados finais esperados para cada comando enviado ao firmware.
    # - OK: comando aceito/concluido
    # - ERROR: firmware retornou erro
    # - TIMEOUT: nao houve resposta no tempo limite
    # - BUSY: firmware ocupado (fila cheia / executando algo)

    class ReplyStatus(Enum):
        OK = 'ok'
        ERROR = 'error'
        TIMEOUT = 'timeout'
        BUSY = 'busy'

    # PARTE 1.2: Niveis de log assicronos emitidos pelo firmware.
    # Esses logs nao sao resposta direta de um comando especifico.
    class LogLevel(Enum):
        DEBUG = 'debug'
        INFO = 'info'
        WARNING = 'warning'
        ERROR = 'error'

    # PARTE 1.3: Prefixos textuais enviados pelo firmware para cada nivel de log.
    # Exemplo: "W) temperatura alta" -> WARNING + " temperatura alta".
    log_level_prefix_map = {
        "D)": LogLevel.DEBUG,
        "I)": LogLevel.INFO,
        "W)": LogLevel.WARNING,
        "E)": LogLevel.ERROR,
    }

    # PARTE 2: Inicializacao da infraestrutura (porta + callbacks + lock/condition + thread de leitura).
    def __init__(self, port: str, baud_rate: int = 115200,
                 command_msg_callback=None,
                 log_msg_callback=None,
                 unsolicited_msg_callback=None,
                 reconnect_timeout: int = 5):
        """
        Inicializa a infraestrutura de comunicacao serial.

        Fluxo geral:
        1) tenta conectar na porta serial;
        2) inicia uma thread dedicada para leitura continua;
        3) para cada linha recebida, classifica como:
           - log,
           - resposta do comando atual,
           - ou mensagem nao solicitada.

        Args:
            port: nome da porta serial (ex: COM3, /dev/ttyACM0).
            baud_rate: taxa de comunicacao.
            command_msg_callback: callback para exibir/registrar comando e resposta.
            log_msg_callback: callback para logs assicronos do firmware.
            unsolicited_msg_callback: callback para mensagens nao associadas a comando.
            reconnect_timeout: tempo de tentativa de reconexao apos perda da serial.
        """
        self.port = port
        self.baud_rate = baud_rate
        self.reconnect_timeout = reconnect_timeout
        self.serial = None  # Objeto serial real sera criado em connect().

        self.command_msg_callback = command_msg_callback
        self.log_message_callback = log_msg_callback
        self.unsolicited_msg_callback = unsolicited_msg_callback

        # Sincronizacao entre:
        # - thread principal (send_command bloqueante)
        # - thread leitora (_reader_loop)
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._waiting_for_response = False
        self._response_string = ""
        self._response_status = None
        self._response_error_msg = None

        self.connect(self.reconnect_timeout)

        # Thread daemon para ler bytes continuamente sem bloquear quem usa a API.
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()


    # PARTE 3: Conecta na serial com timeout e tentativas sucessivas.
    def connect(self, timeout):
        """
        Tenta abrir a porta serial ate o timeout expirar.

        Retorna True ao conectar com sucesso; False se nao foi possivel.
        """
        deadline = time.time() + timeout
        print(Fore.MAGENTA, end='')
        print(f"[SerialInterface] Connecting to port '{self.port}'...", end='')
        while time.time() < deadline:
            try:
                self.serial = serial.Serial(self.port, self.baud_rate, timeout=2)
                print(f" [OK]")
                print(Style.RESET_ALL, end='')
                return True
            except (serial.SerialException, OSError) as e:
                print('.', end='')
                time.sleep(0.2)

        print(f" [FAILED] Timeout after {timeout} seconds.")
        print(f"[SerialInterface] Connection is permanently closed")
        print(Style.RESET_ALL, end='')
        self.serial = None
        return False

    # PARTE 4: Leitura assincrona byte-a-byte e montagem de linhas completas.
    def _reader_loop(self):
        """
        Loop assicrono de leitura da serial.

        O firmware envia texto em linhas. Aqui os bytes sao lidos 1 a 1,
        acumulados em buffer e, ao encontrar '\\n' ou '\\r', a linha completa
        e encaminhada para _handle_line().
        """
        buffer = ""
        while True:
            try:
                if self.serial is not None and self.serial.in_waiting:
                    char = self.serial.read(1).decode('ascii', errors='ignore')
                    # Delimitadores de fim de linha no protocolo textual.
                    if char in ['\n', '\r']:
                        if len(buffer) > 0:
                            self._handle_line(buffer)
                            buffer = ""
                    else:
                        buffer += char
                else:
                    time.sleep(0.001)
            except (serial.SerialException, OSError) as e:
                # Em caso de desconexao, fecha o que restou e tenta reconectar.
                print(Fore.MAGENTA+f"[SerialInterface] Lost connection: {e}"+Style.RESET_ALL)
                try:
                    if self.serial is not None and self.serial.is_open:
                        self.serial.close()
                except Exception:
                    pass

                self.serial = None
                self.connect(self.reconnect_timeout)

    # PARTE 5: Decide o destino de cada linha recebida (log, resposta, espontanea).
    def _handle_line(self, line: str):
        """
        Processa uma unica linha recebida do firmware.

        Prioridades de classificacao:
        1) logs assicronos (D/I/W/E);
        2) resposta do comando em andamento (quando _waiting_for_response=True);
        3) mensagens espontaneas (unsolicited).
        """
        with self._lock:
            log_level, log_msg = self._check_log_msg(line)
            # print(line)  # pode ser habilitado para debug bruto da serial.
            # 1) Linha de log assicrono do firmware.
            if log_level is not None:
                if self.log_message_callback: self.log_message_callback(log_level, log_msg)
            # 2) Linha pertence ao comando que esta aguardando resposta.
            elif self._waiting_for_response:
                line_lower = line.lower()
                if line_lower.startswith("ok"):
                    self._response_status = SerialInterface.ReplyStatus.OK
                elif line_lower.startswith("busy"):
                    self._response_status = SerialInterface.ReplyStatus.BUSY
                elif line_lower.startswith("error"):
                    self._response_status = SerialInterface.ReplyStatus.ERROR
                    parts = line.split(":", 1)
                    self._response_error_msg = parts[1].strip() if len(parts) > 1 else ""

                if self._response_status is not None:
                    # Resposta final identificada: libera quem chamou send_command().
                    self._condition.notify()
                else:
                    # Linha intermediaria da resposta (payload textual).
                    self._response_string += line + '\n'

            # 3) Mensagem nao solicitada (nao era log nem resposta de comando).
            else:
                if self.unsolicited_msg_callback: self.unsolicited_msg_callback(line)

    def _check_log_msg(self, msg: str):
        # Retorna (nivel, conteudo_sem_prefixo) para mensagens D)/I)/W)/E).
        if len(msg) < 2:
            return None, ''
        return self.log_level_prefix_map.get(msg[:2]), msg[2:]

    # PARTE 6: API bloqueante de envio de comando + espera de resposta final.
    def send_command(self, cmd: str, timeout=2) -> tuple[ReplyStatus, str]:
        """
        Envia um comando textual ao firmware e espera resposta final.

        Comportamento bloqueante:
        - escreve o comando na serial;
        - aguarda ate receber status final (ok/busy/error) ou timeout;
        - retorna (ReplyStatus, texto_acumulado_da_resposta).
        """
        with self._lock:
            if not self.serial or not self.serial.is_open:
                return SerialInterface.ReplyStatus.ERROR, 'Serial not open'

            # Limpa estado interno para nova rodada de requisicao/resposta.
            self._waiting_for_response = True
            self._response_string = ""
            self._response_error_msg = ""
            self._response_status = None

            cmd = (cmd.strip() + "\n")
            self.command_msg_callback(cmd, None, '')

            # Envio fisico do comando para o dispositivo.
            self.serial.write(cmd.encode('ascii'))
            self.serial.flush()

            # Aguarda thread leitora sinalizar resposta final.
            end_time = time.time() + timeout
            while self._response_status is None:
                remaining = end_time - time.time()
                if remaining <= 0:
                    self._waiting_for_response = False
                    print(Fore.MAGENTA + f"[SerialInterface] Command timeout, device didn't reply in time" + Style.RESET_ALL)
                    return SerialInterface.ReplyStatus.TIMEOUT, self._response_string
                self._condition.wait(timeout=remaining)

            self._waiting_for_response = False
            self.command_msg_callback(self._response_string, self._response_status, self._response_error_msg)
            return self._response_status, self._response_string

    # PARTE 6.1: Encerramento seguro da porta serial.
    def close(self):
        """Fecha a conexao serial, se estiver aberta."""
        if self.serial and self.serial.is_open:
            self.serial.close()

# --- OpenMicroStageInterface ------------------------------------------------------------------------------------------
# PARTE 7: Camada de alto nivel para uso no projeto (metodos orientados ao dominio).
# Camada de alto nivel da API.
# Traduz chamadas Python (home, move_to, calibrate...) em comandos G/M-code.

class OpenMicroStageInterface:
    # Cores visuais para logs recebidos via callback.
    LOG_COLORS = {
        SerialInterface.LogLevel.DEBUG: Fore.WHITE+Style.DIM,
        SerialInterface.LogLevel.INFO: Style.RESET_ALL,
        SerialInterface.LogLevel.WARNING: Fore.YELLOW,
        SerialInterface.LogLevel.ERROR: Fore.RED,
    }

    # PARTE 8: Estado da interface de alto nivel (serial, transformacao e flags de exibicao).
    def __init__(self, show_communication=True, show_log_messages=True):
        # Transformacao homogenea 4x4 aplicada nas coordenadas de entrada
        # antes de gerar comandos de movimento.
        self.serial = None
        self.workspace_transform = np.eye(4)
        self.show_communication = show_communication
        self.show_log_messages = show_log_messages
        self.disable_message_callbacks = False

    # PARTE 8.1: Handshake inicial (abre serial, registra callbacks e valida versao de firmware).
    def connect(self, port: str, baud_rate: int = 921600):
        # Conversor local para imprimir versao no formato vX.Y.Z.
        def version_to_str(v):
            return f"v{v[0]}.{v[1]}.{v[2]}"

        # Se ja havia conexao, fecha antes de reabrir.
        if self.serial is not None: self.disconnect()
        self.serial = SerialInterface(port, baud_rate,
                                      log_msg_callback=self.log_msg_callback,
                                      command_msg_callback=self.command_msg_callback,
                                      unsolicited_msg_callback=self.unsolicited_msg_callback)

        # Evita poluicao de logs durante handshake inicial.
        self.disable_message_callbacks = True
        fw_version = self.read_firmware_version()
        min_fw_version = (1, 0, 1)
        print(Fore.MAGENTA + f"Firmware version: {version_to_str(fw_version)}" + Style.RESET_ALL)
        # Bloqueia uso se firmware for mais antigo que o minimo suportado.
        if fw_version < min_fw_version:
            print(Fore.MAGENTA + f"Firmware version {version_to_str(fw_version)} incompatible. "
                                 f"At least {version_to_str(min_fw_version)} required" + Style.RESET_ALL)
            self.serial = None
        print('')
        self.disable_message_callbacks = False

    # PARTE 8.2: Desconexao explicita pelo usuario.
    def disconnect(self):
        # Encerramento simples da interface de comunicacao.
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    # PARTE 8.3: Callback de log assicrono.
    def log_msg_callback(self, log_level, msg):
        # Callback para logs assicronos vindos do firmware.
        if not self.show_log_messages or self.disable_message_callbacks:
            return

        color = OpenMicroStageInterface.LOG_COLORS.get(log_level, Fore.WHITE)
        if log_level not in [SerialInterface.LogLevel.INFO, SerialInterface.LogLevel.DEBUG]:
            print(f"{color}[{log_level.name}] {msg}{Style.RESET_ALL}")
        else:
            print(f"{color}{msg}{Style.RESET_ALL}")

    # PARTE 8.4: Callback para comandos enviados e respostas finais.
    def command_msg_callback(self, msg, reply_status: SerialInterface.ReplyStatus, error_msg: str):
        # Callback para trafego de comando/resposta.
        # Quando reply_status is None: mensagem de envio.
        # Quando reply_status existe: resposta final (OK/BUSY/ERROR/TIMEOUT).
        if not self.show_communication or self.disable_message_callbacks:
            return

        if reply_status is not None:
            if msg:
                msg = '\n'.join('> ' + line for line in msg.splitlines())
                print(f"{msg.rstrip()}")
            if error_msg:
                print(f"{Style.BRIGHT}{str(reply_status.name)}:{Style.RESET_ALL} {error_msg}\n")
            else:
                print(f"{Style.BRIGHT}{str(reply_status.name)} {Style.RESET_ALL}\n")
        else:
            print(f"{Fore.GREEN+Style.BRIGHT}{msg.rstrip()}{Style.RESET_ALL}")

    # PARTE 8.5: Callback para mensagens espontaneas do firmware.
    def unsolicited_msg_callback(self, msg):
        # Mensagens espontaneas do firmware (fora do fluxo request/response).
        print(Fore.CYAN+msg+Style.RESET_ALL)
        pass

    # PARTE 8.6: Configuracao da matriz de transformacao de workspace.
    def set_workspace_transform(self, transform):
        # Define matriz 4x4 aplicada em move_to() e set_pose().
        self.workspace_transform = transform

    def get_workspace_transform(self):
        # Le a transformacao atualmente ativa.
        return self.workspace_transform

    def read_firmware_version(self):
        # M58 retorna algo como: "v1.2.3"
        ok, response = self.serial.send_command("M58")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return 0, 0, 0

        # Extrai major/minor/patch por regex.
        major, minor, patch = map(int, re.match(r'v(\d+)\.(\d+)\.(\d+)', response).groups())
        return major,minor,patch

    # PARTE 9: Comandos de movimento e calibracao.
    def home(self, axis_list=None):
        """
        Executa homing (G28) de um ou mais eixos.

        axis_list usa indices 0..5 mapeados para letras A..F.
        Se axis_list=None, envia homing para todos os eixos disponiveis.
        """
        cmd = 'G28'
        axis_chars = ['A', 'B', 'C', 'D', 'E', 'F']
        if axis_list is None:
            axis_list = [i for i in range(len(axis_chars))]

        for axis_idx in axis_list:
            if 0 > axis_idx >= len(axis_chars):
                raise ValueError('Axis index out of range')
            cmd += ' '+axis_chars[axis_idx]

        res, msg = self.serial.send_command(cmd + "\n", 10)
        return res

    def calibrate_joint(self, joint_index: int, save_result: bool):
        """
        Executa calibracao de uma junta (M56) e interpreta tabela retornada.

        Retorno:
            data[0] -> angulo mecanico/motor
            data[1] -> angulo de campo eletrico
            data[2] -> contagem bruta do encoder
        """
        cmd = f"M56 J{joint_index} P"
        if save_result: cmd += ' S'
        res, msg = self.serial.send_command(cmd, 30)

        # Cada linha da resposta contem 3 colunas CSV numericas.
        calibration_data = self._parse_table_data(msg, 3)
        return res, calibration_data

    # PARTE 9.1: Movimento absoluto principal (G0) com politica de retry em BUSY.
    def move_to(self, x, y, z, f, move_immediately=False, blocking=True, timeout=1):
        """
        Move para uma posicao absoluta no espaco de trabalho.

        Passos:
        1) aplica workspace_transform em (x, y, z);
        2) monta comando G0 com feed rate F;
        3) envia e, se BUSY:
           - blocking=True  -> tenta novamente ate aceitar;
           - blocking=False -> retorna BUSY imediatamente.
        """
        # Coordenada homogenea [x, y, z, 1] para permitir translacao/rotacao em matriz 4x4.
        transformed = self.workspace_transform @ np.array([x, y, z, 1.0])
        x_t, y_t, z_t = transformed[:3] / transformed[3]

        cmd = f"G0 X{x_t:.6f} Y{y_t:.6f} Z{z_t:.6f} F{f:.3f}"
        if move_immediately:
            cmd += " I"

        # Reenvia quando o firmware responde BUSY (fila/execucao ocupada).
        while True:
            res, msg = self.serial.send_command(cmd + "\n", timeout=timeout)
            if res != SerialInterface.ReplyStatus.BUSY or not blocking:
                return res

    # PARTE 9.2: Pausa temporal no firmware (G4), tambem com retry em BUSY.
    def dwell(self, time_s, blocking, timeout=1):
        # G4: pausa controlada no firmware por time_s segundos.
        cmd = f"G4 S{time_s:.6f}\n"
        # Mesmo comportamento de retry quando BUSY.
        while True:
            res, msg = self.serial.send_command(cmd + "\n", timeout=timeout)
            if res != SerialInterface.ReplyStatus.BUSY or not blocking:
                return res

    def set_max_acceleration(self, linear_accel, angular_accel):
        # Evita acelercao nula/negativa antes de enviar ao firmware.
        linear_accel = max(linear_accel, 0.01)
        angular_accel = max(angular_accel, 0.01)
        cmd = f"M204 L{linear_accel:.6f} A{angular_accel:.6f}\n"
        res, msg = self.serial.send_command(cmd)
        return res

    # PARTE 10: Leitura de estado, posicao e configuracoes de controle.
    def wait_for_stop(self, polling_interval_ms=10, disable_callbacks=True):
        # Consulta M53 repetidamente ate o firmware indicar "1" (parado).
        # polling_interval_ms esta disponivel na assinatura para uso futuro.
        disable_message_callbacks_prev = self.disable_message_callbacks
        if disable_callbacks: self.disable_message_callbacks = True

        while True:
            res, msg = self.serial.send_command("M53\n")
            if res != SerialInterface.ReplyStatus.OK: return res
            elif msg.strip() == "1":
                return SerialInterface.ReplyStatus.OK

        self.disable_message_callbacks = disable_message_callbacks_prev

    def read_current_position(self):
        # M50 deve retornar algo no formato: X<valor> Y<valor> Z<valor>.
        ok, response = self.serial.send_command("M50")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return None, None, None

        # Aceita com ou sem espaco apos letra do eixo (X-1.0 ou X -1.0).
        match = re.search(
            r"X([-+]?\d*\.?\d+)\s*Y([-+]?\d*\.?\d+)\s*Z([-+]?\d*\.?\d+)",
            response
        )
        if not match:
            raise ValueError(f"Invalid format: {response}")

        x, y, z = match.groups()
        return float(x), float(y), float(z)

    def read_encoder_angles(self):
        # Placeholder: comando existe, mas parse ainda nao implementado.
        ok, response = self.serial.send_command("M51")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return []
        return []

    def read_device_state_info(self):
        # Dispara M57 para o firmware responder estado atual do dispositivo.
        res, msg = self.serial.send_command("M57")
        return res

    def set_servo_parameter(self, pos_kp=150, pos_ki=50000, vel_kp=0.2, vel_ki=100, vel_filter_tc=0.0025):
        # M55 escreve ganhos/filtro do servo no firmware.
        cmd = f"M55 A{pos_kp:.6f} B{pos_ki:.6f} C{vel_kp:.6f} D{vel_ki:.6f} F{vel_filter_tc:.6f}"
        res, msg = self.serial.send_command(cmd)
        return res

    def enable_motors(self, enable):
        # M17 liga estagios de potencia; M18 desliga.
        cmd = f"M17" if enable else "M18"
        res, msg = self.serial.send_command(cmd, timeout=5)
        return res

    def set_pose(self, x, y, z):
        # Mesmo pipeline de transformacao usado em move_to().
        transformed = self.workspace_transform @ np.array([x, y, z, 1.0])
        x_t, y_t, z_t = transformed[:3] / transformed[3]

        cmd = f"G24 X{x_t:.6f} Y{y_t:.6f} Z{z_t:.6f}" # TODO: A, B ,C
        res, msg = self.serial.send_command(cmd)
        return res

    def send_command(self, cmd: str, timeout_s: float=5):
        # Atalho para enviar comando raw sem encapsulamento adicional.
        res, msg = self.serial.send_command(cmd, timeout_s)
        return res, msg

    # PARTE 11: Utilitario para converter resposta tabular (CSV) em listas numericas por coluna.
    @staticmethod
    def _parse_table_data(data_string, cols):
        # Converte string CSV multi-linha em listas por coluna.
        # Ex: cols=3 -> [col0, col1, col2]
        data = [[] for _ in range(cols)]

        for line in data_string.strip().splitlines():
            parts = line.strip().split(',')
            if len(parts) != cols:
                continue  # Ignora linha malformada sem quebrar a coleta inteira.
            numbers = map(float, parts)
            for i, n in enumerate(numbers):
                data[i].append(n)

        return data
