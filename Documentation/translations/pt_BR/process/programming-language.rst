.. SPDX-License-Identifier: GPL-2.0

.. _pt_BR_programming_language:

Linguagem de programação
========================

O kernel Linux é escrito na linguagem de programação C [pt-c-language]_.
Mais precisamente, o kernel costuma ser compilado com ``gcc`` [pt-gcc]_
usando ``-std=gnu11`` [pt-gcc-c-dialect-options]_, que seleciona o dialeto
GNU do padrão ISO C11. ``clang`` [pt-clang]_ também é suportado; consulte
a documentação sobre
:ref:`Compilando o Linux com Clang/LLVM <kbuild_llvm>`.

Esse dialeto contém muitas extensões à linguagem [pt-gnu-extensions]_,
muitas das quais são usadas rotineiramente no kernel.

Atributos
----------

Os atributos [pt-gcc-attribute-syntax]_ estão entre as extensões mais
comuns usadas em todo o kernel. Eles permitem introduzir semântica
definida pela implementação em entidades da linguagem (como variáveis,
funções ou tipos), sem exigir mudanças sintáticas significativas
na linguagem (por exemplo, a adição de uma nova palavra-chave) [pt-n2049]_.

Em alguns casos, os atributos são opcionais (isto é, um compilador que
não os suporte ainda deve gerar código correto, mesmo que o programa
resultante seja mais lento ou o compilador realize menos verificações e
diagnósticos em tempo de compilação).

O kernel define pseudopalavras-chave (por exemplo, ``__pure``) em vez de
usar diretamente a sintaxe de atributos do GNU (por exemplo,
``__attribute__((__pure__))``), para detectar quais atributos são
suportados e/ou encurtar o código.

Consulte ``include/linux/compiler_attributes.h`` para mais informações.

Rust
----

O kernel oferece suporte à linguagem de programação Rust [pt-rust-language]_
quando a opção ``CONFIG_RUST`` está habilitada. O código Rust é compilado
com ``rustc`` [pt-rustc]_ usando ``--edition=2021`` [pt-rust-editions]_. As
edições permitem introduzir pequenas mudanças na linguagem que não são
compatíveis com versões anteriores.

Além disso, alguns recursos instáveis [pt-rust-unstable-features]_ são usados
no kernel. Esses recursos podem mudar no futuro; portanto, é importante
chegar a um ponto em que apenas recursos estáveis sejam usados.

Consulte Documentation/rust/index.rst para mais informações.

.. [pt-c-language] http://www.open-std.org/jtc1/sc22/wg14/www/standards
.. [pt-gcc] https://gcc.gnu.org
.. [pt-clang] https://clang.llvm.org
.. [pt-gcc-c-dialect-options] https://gcc.gnu.org/onlinedocs/gcc/C-Dialect-Options.html
.. [pt-gnu-extensions] https://gcc.gnu.org/onlinedocs/gcc/C-Extensions.html
.. [pt-gcc-attribute-syntax] https://gcc.gnu.org/onlinedocs/gcc/Attribute-Syntax.html
.. [pt-n2049] http://www.open-std.org/jtc1/sc22/wg14/www/docs/n2049.pdf
.. [pt-rust-language] https://www.rust-lang.org
.. [pt-rustc] https://doc.rust-lang.org/rustc/
.. [pt-rust-editions] https://doc.rust-lang.org/edition-guide/editions/
.. [pt-rust-unstable-features] https://github.com/Rust-for-Linux/linux/issues/2
