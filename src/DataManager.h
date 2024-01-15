/*
O DataManager deve ser uma classe que lida com todos os BOs
de enviar/receber dados de maneira confiável e robusta

Ele recebe os objetos de tudo o que tem a ver com dados: 
 - SDCard
 - MQTT
 - Modem

Deve ter o método:

- "SendData (string/json/sla...)"

    A Send data checa se há conexão disponível. Se sim, manda os dados pro MQTT normalmente.
    Se não tem conexão disponível, checa se os dados são com GPS ou não, adiciona no payload 
    o total de espaço ainda disponível na mídia e guarda no arquivo correto (com ou sem GPS)


- "loop/update/run/sla..." que funciona da mesma maneira do mqtt.loop
se eu quiser ser mto mala mesmo, eu transformo ele em uma task e jogo esse trabalho pro OS

    a função loop é encarregada de garantir que todos os dados dos arquivos foram enviados 
    e depois limpos do cartão.

*/


// Uma outra ideia é escrever arquivos de até N mensagens. 
// A cada loop, checa se tem arquivos faltando e manda eles na ordem normal mesmo.


#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SdFat.h>
#include <StreamUtils.h>

class DataManager {
  public:
    bool isConnected = false;
    bool sdCardAvailable = false;
    DataManager(SdFat SD, PubSubClient* mqtt, const char* mqttTopic, const char* gpsFilename, const char* offlineFilename){
        this->mqtt = mqtt;
        this->SD = SD;
        this->mqttTopic = mqttTopic;
        this->gpsFilename = gpsFilename;
        this->offlineFilename = offlineFilename;
        loop();
    }
    int sendMessage(JsonDocument message){
        loop();
        int messageSent = 0;
        if(isConnected){
           messageSent = sendMqttMessage(message);
        }
        else if((!isConnected || !messageSent) && sdCardAvailable){
            uint32_t freeKB = SD.vol()->freeClusterCount();
            freeKB *= SD.vol()->sectorsPerCluster()/2;
            message["spaceLeftKb"] = freeKB;
            if(message["gps"]){
                messageSent = addMessageToFile(message, gpsFilename);
            }else{
                messageSent = addMessageToFile(message, offlineFilename);
            }
        }else{
          Serial.println("SDCARD not available or another error ocurred");
        }
        return messageSent;
    };
    int loop(){
        // checks if is connected
        isConnected = mqtt->connected();
        if(isConnected){
            //check for pending data
            if(check_filesize(gpsFilename) > 0){
              handleFileContent(gpsFilename, 15);
            }
            if(check_filesize(offlineFilename) > 0){
              handleFileContent(offlineFilename, 15);
            }
        }
        return 1;
    };

  private:
    const char *mqttTopic, *gpsFilename, *offlineFilename;
    PubSubClient* mqtt;
    SdFat SD;
    
    int sendMqttMessage(JsonDocument message){
        String output;
        serializeJson(message, output);
        return (int)mqtt->publish(mqttTopic, output.c_str());
    };
    int addMessageToFile(JsonDocument message, const char *fileName){
        SdFile file;
        if (!file.open(fileName, O_WRONLY | O_CREAT | O_AT_END)) {
          Serial.printf("addMessageToFile CouldNot open file '%s' for writing\n", fileName);
          return 0;
        }
        size_t written = 0;
        int size_before = file.fileSize();
        if(size_before > 0){
          written+= file.write('\n');
        }else{
          Serial.println("writing to empty file");
        }
        // String output;
        written += serializeJson(message, file);
        int size_after = file.fileSize();
        Serial.printf("%s -> Wrote %d bytes to file. Growing from %d to %d bytes\n", fileName, written, size_before, size_after);
        file.close();
        return (int)written > 0;
    };

    int check_filesize(const char* fileName){
      SdFile file;
      if (!file.open(fileName, O_RDONLY)) {
        // Serial.printf("check_filesize Could Not open '%s'\n", fileName);
        return 0;
      }
      int size = file.fileSize();
      file.close();
      return size;
    }

    void errorPrint() {
      if (SD.sdErrorCode()) {
        Serial.print(F("SdError: 0X"));
        Serial.print(SD.sdErrorCode(), HEX);
        Serial.print(F(",0X"));
        Serial.println(SD.sdErrorData(), HEX);
      } else if (!SD.fatType()) {
        Serial.println(F("Check SD format."));
      }
    }

    int handleFileContent(const char* fileName, int maxMessages){
      /*
       A cada chamada de "handleFileContent" ele deve:
        - ler as mensagens do final até o início
        - caso tenha chegado no início do arquivo ou atingido maxMessages
          inicia a cópia do que ainda resta do arquivo para um novo arquivo
        - deleta o arquivo inicial
        - renomeia o novo arquivo de volta para fileName
      */
      char tmpFilename[100];
      SdFile file;
      if (!file.open(fileName, O_RDWR)) {
        Serial.printf("HFC CouldNot open file '%s' for writing\n", fileName);
        return -1;
      }
      // << repeat
      // bota a cabeça pro final
      file.seekEnd();
      int lastLineEnd = file.curPosition()-1;
      file.seekSet(lastLineEnd);
      int lastLineStart;
      int msgsRead = 0;
      // char c = file.peek();
      Serial.printf("HFC at position %d\n", lastLineEnd);
      // enquanto não chegar no máximo de msgs >> 
      while(msgsRead < maxMessages && lastLineEnd > 0){
      //   encontra o inicio da ultima mensagem '{'
        lastLineStart = findLastLineStart(file);
        if(lastLineStart<0){
          Serial.print("FindLastLine could not read file");
        }
        // c = file.peek();
        // Serial.printf("  [last line start] at position %d and char %d\n", lastLineStart, c);
      //   transmite a última mensagem '{...}'
        sendMessageFromFile(file, lastLineStart, lastLineEnd);
        msgsRead++;
      //   volta a cabeça pro começo da ultima mensagem (ou seja, volta 2 na posição)
      // '} <- \n <- {'
        lastLineEnd = lastLineStart - 2;
        if(lastLineEnd < 0) lastLineEnd = 0;
        file.seekSet(lastLineEnd);
        // c = file.peek();
        // Serial.printf("  [last line end] at position %d and char %d\n", lastLineEnd, c);
      }
      Serial.printf("Sent %d messages. Reading ended at %d. ", msgsRead, lastLineEnd);

      // se chegou no início do arquivo, deleta
      if(lastLineEnd == 0){
        Serial.println("All messages were sent. Deleting file");
        file.remove();
      }else{ // se não chegou no final do arquivo:
        SdFile tempfile;
        sprintf(tmpFilename, "tmp_%s", fileName);
        if (!tempfile.open(tmpFilename, O_RDWR | O_CREAT)) {
          Serial.printf("CouldNot open temp file '%s' for copying", tmpFilename);
          file.close();
          return -1;
        }
        // TODO: Ele não copia os dados pro novo arquivo temporário
        // copia tudo do início até onde foi lido para um arquivo temporário
        Serial.printf("Copying remaining messages to new file '%s'...\n", tmpFilename);
        copyFileUpToPoint(file, tempfile, lastLineEnd);
        tempfile.sync();
        Serial.println("Done");
        //  deleta o arquivo original
        if(!file.remove()){
          Serial.println("Failed to remove original File");
          file.close();
          return -1;
        }
        file.close();
        tempfile.close();
        //  renomeia o arquivo temporario
        if (!tempfile.open(tmpFilename, O_WRONLY)) {
          Serial.printf("CouldNot open temp file '%s' for renaming", tmpFilename);
          return -1;
        }else{
          if(!tempfile.rename(fileName)){
            Serial.print("Could not rename tempFile back to original name");
          }
          tempfile.close();
        }
      }
      return 1;
    }

    int findLastLineStart(SdFile file){
      // retorna a posição do início da mensagem anterior (ou seja, do \n anterior)
      const int chunkSize = 64;
      char buff[chunkSize];
      int currPos = file.curPosition();
      int bytesToRead = chunkSize;
      int msgStart = 0;
      // Enquanto não acabou o arquivo
      while(currPos > 0){
        if(currPos < chunkSize){
          bytesToRead = currPos + 1; 
        }
        // else{
        //   bytesToRead = chunkSize;
        // }
        currPos -= bytesToRead - 1;
        file.seekSet(currPos);
        if(!file.read(buff, bytesToRead)) return -1;
        for(int i = bytesToRead - 1; i >= 0; i--){
          if(buff[i] == '\n'){
            msgStart = currPos + i + 1; // o start é no primeiro caractere DEPOIS do \n
            file.seekSet(msgStart);
            return msgStart;
          }
        }
        currPos--; //Lemos tudo, agora vamos pro próximo chunk
      }
      return msgStart; // se chegou até aqui, é pq o start é 0
    }

    int sendMessageFromFile(SdFile file, int startPosition, int endPosition){
      char buff[513]; //this is the maximum message size. More than that and it will be ignored 
      int bytesToRead = endPosition - startPosition + 1;
      if(bytesToRead > 512) bytesToRead = 512;
      file.seekSet(startPosition);
      file.read(buff, bytesToRead);
      buff[bytesToRead] = 0;
      int success = mqtt->publish(mqttTopic, buff, 0); //not retained
      // file.seekSet(startPosition);
      // file.read(buff, bytesToRead);
      // Serial.printf("Reading %d bytes from %d to %d\nSending MQTT to \"%s\" -> ", bytesToRead, startPosition, endPosition, mqttTopic);
      // Serial.write(buff, bytesToRead);
      // Serial.println("||");
      return success;
    }

    void copyFileUpToPoint(SdFile origin, SdFile destination, int endPosition){
      char buff[64];
      int bytesToRead = 64;
      int currPosition = 0;
      int totalWritten = 0;
      origin.seekSet(currPosition);
      destination.seekSet(0);
      // Serial.println("Dumping File content ->");
      while(currPosition < endPosition){
        if(endPosition - currPosition < bytesToRead){
          bytesToRead = endPosition - currPosition + 2;
        }
        size_t bytesRead = origin.read(buff,bytesToRead);
        // Serial.write(buff, bytesRead);
        totalWritten += destination.write(buff, bytesRead);
        currPosition+=bytesRead;
      }
      destination.sync();
    }
};

#endif


