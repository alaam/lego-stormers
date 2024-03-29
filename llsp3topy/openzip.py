#PYSPIKON - PySpikeConverter

# ZIP UNMASKER - EXTRAIDOR DE ZIP
# pyjonhact - 2020 | License GNU GPL v3

# Importa blibliotecas
from zipfile import ZipFile
import shutil
import os

# Define a função de extração.


def extractandopen(directory, origin):
    # Copia o arquivo como .zip e extrai o arquivo scratch.sb3
    shutil.copyfile(directory, directory.replace(origin, '.zip'))
    with ZipFile(directory.replace(origin, '.zip'), 'r') as zip:
        zip.extract('scratch.sb3')
    # Deleta o arquivo
    os.remove(directory.replace('.llsp', '.zip'))
    # Repete o mesmo processo com o arquivo scratch.sb3
    with ZipFile('scratch.sb3', 'r') as zip:
        zip.extract('project.json')
    os.remove('scratch.sb3')
    # Retorna o arquivo project.json
    return open('project.json', encoding='utf-8')

def extractandopen3(directory, origin):
    # Copia o arquivo como .zip e extrai o arquivo scratch.sb3
    shutil.copyfile(directory, directory.replace(origin, '.zip'))
    with ZipFile(directory.replace(origin, '.zip'), 'r') as zip:
        zip.extract('projectbody.json')
    # Deleta o arquivo
    os.remove(directory.replace('.llsp3', '.zip'))
    # Retorna o arquivo project.json
    jso=open('projectbody.json', encoding='utf-8')
    #os.remove('projectbody.json')
    return jso
