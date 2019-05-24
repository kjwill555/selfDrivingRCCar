from tensorflow.python.keras.layers import Input
from tensorflow.python.keras.models import Model, load_model
from tensorflow.python.keras.layers import Conv2D, Dense
from tensorflow.python.keras.layers import Dropout, Flatten, MaxPooling2D
from tensorflow.python.keras.layers import Activation, BatchNormalization
from tensorflow.python.keras.callbacks import ModelCheckpoint, EarlyStopping


class AutoPilot:

    """
    The AutoPilot model

    Parameters:
    model_path (str) : Path to an existing model. If None, a new default model
        is used
    """
    def __init__(self, model_path=None):
        if model_path is not None:
            self.load(model_path)
        else:
            self.model = default_model()


    """
    Load a model from the disk

    Parameters:
    model_path (str) : The path to the model
    """
    def load(self, model_path):
        self.model = load_model(model_path)


    """
    Train the model

    Parameters:
    train_gen (generator) : Training data generator
    val_gen (generator) : Validatoin data generator
    saved_model_path (str) : Where to save the model
    epochs (int) : Number of epochs to train
    steps (int) : Steps per epoch
    train_split (float) : The train/val split
    verbose (int) : Verbosity level
    min_delta (float) : Minimum val_loss delta for early stopping
    patience (int) : Number of epochs without improvement to stop at
    use_early_stop (bool) : Whether to use automatic early stopping

    Returns:
    history : The training history of the model
    """
    def train(self, train_gen, val_gen,
              saved_model_path, epochs=100, steps=100, train_split=0.8,
              verbose=1, min_delta=.0005, patience=5, use_early_stop=True):

        # Checkpoint to save model after each epoch
        save_best = ModelCheckpoint(saved_model_path,
                                    monitor='val_loss',
                                    verbose=verbose,
                                    save_best_only=True,
                                    mode='min')

        # Stop training if the validation error stops improving.
        early_stop = EarlyStopping(monitor='val_loss',
                                   min_delta=min_delta,
                                   patience=patience,
                                   verbose=verbose,
                                   mode='auto')

        callbacks_list = [save_best]

        if use_early_stop:
            callbacks_list.append(early_stop)

        hist = self.model.fit_generator(
            train_gen,
            steps_per_epoch=steps,
            epochs=epochs,
            verbose=1,
            validation_data=val_gen,
            callbacks=callbacks_list,
            validation_steps=steps * (1.0 - train_split) / train_split)
        return hist


    """
    Make a prediction

    Parameter:
    img_arr (nparray) : The image from which to make a prediction

    Returns:
    float, float : The predicted steering value, the predicted throttle value
    """
    def predict(self, img_arr):
        img_arr = img_arr.reshape((1,) + img_arr.shape)
        outputs = self.model.predict(img_arr)
        steering = outputs[0]
        throttle = outputs[1]
        return steering[0][0], throttle[0][0]


""" Creates a default model """
def default_model():
    img_in = Input(shape=(120, 160, 1), name='img_in')
    x = img_in

    x = Conv2D(filters=32, kernel_size=(3, 3)) (x)
    x = BatchNormalization(axis=-1) (x)
    x = Activation("relu") (x)
    x = Conv2D(filters=32, kernel_size=(3, 3)) (x)
    x = BatchNormalization(axis=-1) (x)
    x = Activation("relu") (x)
    x = MaxPooling2D(pool_size=(2,2)) (x)

    x = Conv2D(filters=64, kernel_size=(3, 3)) (x)
    x = BatchNormalization(axis=-1) (x)
    x = Activation("relu") (x)
    x = Conv2D(filters=64, kernel_size=(3, 3)) (x)
    x = BatchNormalization(axis=-1) (x)
    x = Activation("relu") (x)
    x = MaxPooling2D(pool_size=(2,2)) (x)

    x = Flatten() (x)
    x = Dense(units=512) (x)
    x = BatchNormalization() (x)
    x = Activation("relu") (x)
    x = Dropout(rate=.2) (x)
    x = Dense(units=50, activation="relu") (x)

    angle_out = Dense(units=1, activation="relu", name='angle_out') (x)

    throttle_out = Dense(units=1, activation='relu', name='throttle_out') (x)

    model = Model(inputs=[img_in], outputs=[angle_out, throttle_out])

    model.compile(optimizer='adam',
                  loss={'angle_out': 'mean_squared_error',
                        'throttle_out': 'mean_squared_error'},
                  loss_weights={'angle_out': 0.5, 'throttle_out': .5})

    return model
