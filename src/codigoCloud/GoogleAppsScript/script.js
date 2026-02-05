
// Esta função é executada automaticamente quando o ESP32 envia dados via HTTP POST
function doPost(e) {
  var sheet = SpreadsheetApp.getActiveSpreadsheet(); // Abre a planilha atual

  // Converte os dados recebidos (em texto JSON) para um objeto JavaScript
  var params = JSON.parse(e.postData.contents);
  var action = params.action; // Identifica qual ação o ESP32 quer realizar

  // Verifica qual ação foi pedida e chama a função correspondente
  if (action === "escreverEmLista") {
    return escreverEmLista(sheet, params);
  } else if (action === "escreverEmCelula") {
    return escreverEmCelula(sheet, params);
  } else if (action === "lerCelula") {
    return lerCelula(sheet, params);
  } else if (action === "lerLinha") {
    return lerLinha(sheet, params);
  } else {
    return ContentService.createTextOutput("Ação não reconhecida");
  }
}

// Escreve os dados em uma nova linha na aba da planilha informada
function escreverEmLista(sheet, params) {
  var identificacao = params.identificacao; // Nome da aba (ex: "dados")
  var dados = params.dados; // Lista com os valores enviados (ex: [23.5, 70])

  // Procura pela aba com o nome desejado ou cria uma nova se não existir
  var aba = sheet.getSheetByName(identificacao) || sheet.insertSheet(identificacao);
  var ultimaLinha = aba.getLastRow() + 1; // A próxima linha vazia
  var dataHora = new Date(); // Data e hora atuais

  // Coluna 1: Data e hora juntas (formato completo do Google Sheets)
  aba.getRange(ultimaLinha, 1).setValue(dataHora);

  // Coluna 2: Só a data, em formato local (ex: "28/05/2025")
  aba.getRange(ultimaLinha, 2).setValue(dataHora.toLocaleDateString());

  // Coluna 3: Só a hora, em formato local (ex: "14:22:10")
  aba.getRange(ultimaLinha, 3).setValue(dataHora.toLocaleTimeString());

  // A partir da coluna 4, escreve todos os valores enviados pelo ESP32
  for (var i = 0; i < dados.length; i++) {
    aba.getRange(ultimaLinha, i + 4).setValue(dados[i]);
  }

  // Envia uma resposta para o ESP32 indicando que deu tudo certo
  return ContentService.createTextOutput("Dados salvos com sucesso");
}

// Escreve um único valor em uma célula específica da aba
function escreverEmCelula(sheet, params) {
  var identificacao = params.identificacao; // Nome da aba
  var celula = params.celula; // Exemplo: "B2"
  var dado = params.dado; // Valor a ser colocado na célula

  var aba = sheet.getSheetByName(identificacao) || sheet.insertSheet(identificacao);

  aba.getRange(celula).setValue(dado); // Escreve o dado na célula desejada
  return ContentService.createTextOutput("Dado salvo na célula " + celula);
}

// Lê e retorna o valor de uma célula específica da aba
function lerCelula(sheet, params) {
  var identificacao = params.identificacao; // Nome da aba
  var celula = params.celula; // Exemplo: "C2"

  var aba = sheet.getSheetByName(identificacao);

  if (!aba) {
    return ContentService.createTextOutput("Aba não encontrada");
  }

  var valor = aba.getRange(celula).getValue(); // Lê o valor da célula
  return ContentService.createTextOutput(valor); // Retorna o valor para o ESP32
}

// Lê todos os valores de uma linha da aba, formatando a data e hora
function lerLinha(sheet, params) {
  try {
    Logger.log("Iniciando função lerLinha");

    var identificacao = params.identificacao; // Nome da aba (ex: "dados")
    var linha = params.linha; // Número da linha a ser lida (ex: 2)

    Logger.log("Identificação: " + identificacao);
    Logger.log("Linha: " + linha);

    var aba = sheet.getSheetByName(identificacao); // Procura a aba

    if (!aba) {
      Logger.log("Aba não encontrada: " + identificacao);
      return ContentService.createTextOutput("Aba não encontrada");
    }

    Logger.log("Aba encontrada: " + identificacao);
	
	// Verifica se a linha solicitada existe
    var ultimaLinha = aba.getLastRow();
    if (linha > ultimaLinha || linha < 1) {
      Logger.log("Linha inválida: " + linha + " (máximo permitido: " + ultimaLinha + ")");
      return ContentService.createTextOutput("Linha inválida: " + linha);
    }

    // Pega todos os valores da linha, até a última coluna com dados
    var valores = aba.getRange(linha, 1, 1, aba.getLastColumn()).getValues()[0];

    // Formata a data (coluna 1) no padrão "dd/MM/yyyy"
    var dataFormatada = Utilities.formatDate(new Date(valores[0]), "GMT-3", "dd/MM/yyyy");
    
    // Formata a hora (coluna 2) no padrão "HH:mm:ss"
    var horaFormatada = Utilities.formatDate(new Date(valores[1]), "GMT-3", "HH:mm:ss");

    // Substitui os valores originais pela data e hora formatadas
    valores[0] = dataFormatada;
    valores[1] = horaFormatada;

    Logger.log("Valores lidos da linha " + linha + ": " + valores.join(";"));

    // Retorna os valores como uma string separada por ponto e vírgula
    return ContentService.createTextOutput(valores.join(";"));
  } catch (e) {
    Logger.log("Erro na função lerLinha: " + e.toString());
    return ContentService.createTextOutput("Erro: " + e.toString());
  }
}



/*
function lerLinha(sheet, params) {
  try {
    var linha = params.linha;
    var aba = sheet.getSheetByName("dados");
    
    if (!aba) {
      return ContentService.createTextOutput("Aba não encontrada");
    }
    
    var valores = aba.getRange(linha, 1, 1, aba.getLastColumn()).getValues()[0];
    
    var dataFormatada = Utilities.formatDate(new Date(valores[0]), "GMT-3", "dd/MM/yyyy");
    var horaFormatada = Utilities.formatDate(new Date(valores[0]), "GMT-3", "HH:mm:ss");
    
    valores[0] = dataFormatada;
    valores[1] = horaFormatada;
    
    return ContentService.createTextOutput(valores.join(";"));
  } catch (e) {
    return ContentService.createTextOutput("Erro: " + e.toString());
  }
}
*/