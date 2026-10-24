.. SPDX-License-Identifier: GPL-2.0

IA Assistente de código
+++++++++++++++++++++++

Esta documentação fornece um guia para ferramentas de IA e desenvolvedores que
usam IA como assistente para contribuir para o kernel do Linux.

Ferramentas de IA que ajudam desenvolvedores do Linux kernel devem seguir os
padrões de desenvolvimento do kernel:

* Documentation/process/development-process.rst
* Documentation/process/coding-style.rst
* Documentation/process/submitting-patches.rst

Para diretrizes sobre conteúdo gerado por assistentes de IA, veja:

* Documentation/process/generated-content.rst

Licenças e Requisitos Legais
============================

Todas as contribuições devem estar de acordo com os requisitos de licença do
kernel:

* Todo código deve ser compatível com GPL-2.0-only
* Use identificadores SPDX de licença apropriados
* Veja Documentation/process/license-rules.rst para mais detalhes

Assinatura e certificado de origem do desenvolvedor
===================================================

Agentes de IA NÃO DEVEM adicionar tags Signed-off-by. Apenas pessoas
podem legalmente certificar o Certificado de Origem do Desenvolvedor (DCO).
A pessoa que envia é responsável por:

* Revisar todo código gerado por IA
* Garantir conformidade com os requisitos de licença
* Acrescentar sua própria tag Signed-off-by para certificar o DCO
* Assumir toda responsabilidade pela contribuição

Procedimento para encontrar e corrigir bugs
===========================================

Quando um assistente de IA é usado para encontrar e corrigir bugs, ele **DEVE**
seguir pelo menos estes passos:

1. Antes de começar, leia toda a documentação do processo listada acima, bem
   como qualquer outro documento mencionado na solicitação. Não confie em
   trechos isolados encontrados por busca de palavras-chave.
2. Anote o ID do commit e localize um bug conforme instruído.
3. Para qualquer bug encontrado que não seja trivial, verifique se ele parece
   real tentando criar um reprodutor que o demonstre. A falta dele pode fazer
   com que o relatório seja ignorado, já que muitos relatórios de bugs não
   verificados enviados aos mantenedores acabam sendo inválidos. Pare aqui se,
   no fim, ele parecer equivocado.
4. Escreva uma correção para o bug. Esta parte não é opcional: exceto em
   alguns poucos casos muito raros, um assistente de IA capaz de encontrar um
   bug é capaz de corrigi-lo. Note que correções escritas na mesma sessão
   usada para encontrar o bug geralmente resultam em correções melhores e mais
   precisas, pois o contexto de raciocínio do LLM continua presente.
5. Compile e verifique se a correção funciona, usando o reprodutor ou
   executando novamente uma análise completa; descarte qualquer correção que
   não funcione e tente outra. A correção não deve adicionar avisos de
   compilação e deve passar nas verificações do checkpatch.pl (veja
   submitting-patches.rst).
6. Faça o commit da correção que funciona com uma mensagem detalhada
   descrevendo o problema, a solução e uma tag Fixes. Não adicione uma tag
   Signed-off-by.
7. Identifique os mantenedores e as listas usando scripts/get_maintainer.pl.
   Documentation/process/security-bugs.rst mostra como fazer isso.
8. Indique o que não pôde ser feito. Se a correção não pôde ser compilada ou
   testada, ou se não foi possível produzir um reprodutor, diga isso
   explicitamente: atualmente os mantenedores perdem tempo demais analisando
   relatórios não verificados e correções não testadas.
9. Leia Documentation/process/threat-model.rst para determinar se o bug é uma
   vulnerabilidade ou um bug comum, e deixe o resultado para a revisão do
   relator (o assistente nunca deve enviar nada por conta própria). Bugs
   comuns são enviados conforme descrito em
   Documentation/process/submitting-patches.rst, e vulnerabilidades conforme
   descrito em Documentation/process/security-bugs.rst.
